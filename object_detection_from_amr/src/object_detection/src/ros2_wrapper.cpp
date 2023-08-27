/*
 INTEL CONFIDENTIAL
 Copyright 2022 Intel Corporation
 This software and the related documents are Intel copyrighted materials, and your use of them is governed by the
 express license under which they were provided to you (License). Unless the License provides otherwise, you may not
 use, modify, copy, publish, distribute, disclose or transmit this software or the related documents without Intel's
 prior written permission.
 This software and the related documents are provided as is, with no express or implied warranties, other than those
 that are expressly stated in the License.
*/


#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "ros2_wrapper.h"

#define REALSENSE_MAX_RANGE 6 //meters - default settings by Realsense d400 series

static rclcpp::QoS getRealSenseQoS()
{
    rmw_qos_reliability_policy_t reliability_policy_ = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
    size_t depth_ = rmw_qos_profile_default.depth;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            history_policy_,
            depth_));
    qos.reliability(reliability_policy_);

    return qos;
}

// initialization of static members of tracked obstacles...
int TrackedObject::s_fade_counter_size = 0;
double TrackedObject::s_sampling_time = 100.0;
double TrackedObject::s_process_variance = 0.0;
double TrackedObject::s_process_rate_variance = 0.0;
double TrackedObject::s_measurement_variance = 0.0;

ROS2Wrapper::ROS2Wrapper() : Node("object_detection_node")
{
    is_new_image = false;
    is_first_image = true;
    id_counter = -1;
    topic_objects_output = std::string("/camera/detected_objects");
    topic_TrackedObjects_output = std::string("/camera/tracked_objects");
    topic_TrackedImage_output = std::string("/camera/image_tracked");
    topic_depth_image = std::string("/camera/aligned_depth_to_color/image_raw");
    topic_camera_info = std::string("/camera/aligned_depth_to_color/camera_info");
    topic_image_input = std::string("/camera/color/image_raw");
    topic_image_remote = std::string("/object/image_for_remote");
    topic_remote_obj_detection = std::string("/object/remote_results");
    topic_switch_to_remote_inf = std::string("/object/enable_remote_inference");


    //tracker's parameters
    p_sampling_time = 30.0;
    p_min_correspondence_cost = 100.0;
    p_loop_rate = 30.0;
    p_tracking_duration = 0.5;
    p_process_variance = 1.0;
    p_process_rate_variance = 10.0;
    p_measurement_variance = 10000.0;

    p_fade_counter_size = p_loop_rate * p_tracking_duration;

    // Inference Engine's parameters
    std::string path = ament_index_cpp::get_package_share_directory("object_detection");
    this->declare_parameter("model_path", std::string(path + "/models/ssd_mobilenet_v2_coco/frozen_inference_graph.xml"));
    this->declare_parameter("device", std::string("CPU"));
    this->declare_parameter("remote_inference_enable", false);

    getParameters();

    initROSInterface();

    objectdetection_.initInferenceEngine(device, model_path);

    setParameters();

    RCLCPP_INFO(get_logger(), "Start Tracking Bounding Boxes");
}

ROS2Wrapper::~ROS2Wrapper()
{
}

void ROS2Wrapper::runTracking()
{

    getTrackedParams();
    publishObjects();
    updateTrackedObjects();
    if (!depthImage_.empty())
    {
        getDistance();
        updateMarkers();
    }

    else
    {
        RCLCPP_WARN(get_logger(), "No depth data received, the node will not publish the position of the detected objects");
    }
}

void ROS2Wrapper::updateMarkers()
{
    visualization_msgs::msg::MarkerArray oldMarkers;
    oldMarkers.markers.resize(1);
    oldMarkers.markers[0].action = visualization_msgs::msg::Marker::DELETEALL;
    oldMarkers.markers[0].header = getHeader();

    /* delete old markers */
    markerPublisher_->publish(oldMarkers);

    /* publish updated markers */
    if (markers.markers.size() > 0)
        markerPublisher_->publish(markers);
}

void ROS2Wrapper::getParameters()
{
    RCLCPP_INFO(this->get_logger(), "[ObjectDetection] Getting parameters ...");

    // Inference Engine's parameters
    std::string path = ament_index_cpp::get_package_share_directory("object_detection");

    get_parameter("model_path", model_path);
    RCLCPP_INFO(this->get_logger(), "Set model_path to: %s\n", model_path.c_str());
    get_parameter("device", device);
    RCLCPP_INFO(this->get_logger(), "Set device to: %s\n", device.c_str());
    get_parameter("remote_inference_enable", remote_inference_enable);
}

void ROS2Wrapper::initROSInterface()
{
    RCLCPP_INFO(get_logger(), "Publishing to %s", topic_objects_output.c_str());
    pub_objects = create_publisher<object_detection::msg::Objects>(topic_objects_output, 1);

    RCLCPP_INFO(get_logger(), "Publishing to %s", topic_TrackedImage_output.c_str());
    pub_image_tracked = create_publisher<sensor_msgs::msg::Image>(topic_TrackedImage_output, 1);

    RCLCPP_INFO(get_logger(), "Publishing to %s", topic_TrackedObjects_output.c_str());
    pub_tracked_objects = create_publisher<object_detection::msg::Objects>(topic_TrackedObjects_output, 1);

    RCLCPP_INFO(get_logger(), "Publishing to %s", topic_image_remote.c_str());
    pub_remote_infer = image_transport::create_publisher(this, topic_image_remote);

    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_image_input.c_str());
    auto qos = getRealSenseQoS();
    sub_image_rgb = create_subscription<sensor_msgs::msg::Image>(
        topic_image_input, qos, std::bind(&ROS2Wrapper::rosCallbackImage, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_depth_image.c_str());
    depthSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_depth_image, qos, std::bind(&ROS2Wrapper::depthCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_camera_info.c_str());
    cameraInfoSubscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic_camera_info, qos, std::bind(&ROS2Wrapper::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_remote_obj_detection.c_str());
    sub_remote_det_objects = this->create_subscription<object_detection::msg::Objects>(
        topic_remote_obj_detection, qos, std::bind(&ROS2Wrapper::remoteDetObjCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic_switch_to_remote_inf.c_str());
    sub_switch_inference_type = this->create_subscription<std_msgs::msg::UInt32>(
        topic_switch_to_remote_inf, qos, std::bind(&ROS2Wrapper::switchToRemoteInfCallback, this, std::placeholders::_1));

    markerPublisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("detected_objects_marker_array", 1);
}


void ROS2Wrapper::updateTrackedObjects()
{
    for (uint32_t i = 0; i < tracked_objects.size(); ++i)
    {
        if (!tracked_objects[i].hasFaded())
        {
            tracked_objects[i].predictState();
        }
        else
        {
            tracked_objects.erase(tracked_objects.begin() + i--);
        }
    }
}

void ROS2Wrapper::setParameters()
{
    TrackedObject::setSamplingTime(p_sampling_time);
    TrackedObject::setCounterSize(p_fade_counter_size);
    TrackedObject::setCovariances(p_process_variance, p_process_rate_variance, p_measurement_variance);
}

double ROS2Wrapper::objectCostFunction(const object_detection::msg::Object &new_object, const object_detection::msg::Object &old_object)
{

    double label_cost = (new_object.label != old_object.label) ? 1 : 0;

    cv::Rect new_rect = cv::Rect(new_object.x, new_object.y, new_object.height, new_object.width);
    cv::Rect old_rect = cv::Rect(old_object.x, old_object.y, old_object.height, old_object.width);
    cv::Rect union_rect = new_rect | old_rect;
    cv::Rect intersect_rect = new_rect & old_rect;
    double iou_cost = 1.0 - (double(intersect_rect.area()) / double(union_rect.area()));

    double cost = 1000 * label_cost + 100 * iou_cost;
    return cost;
}

void ROS2Wrapper::calculateCostMatrix(const std::vector<object_detection::msg::Object> &new_objects, MatrixXd &cost_matrix)
{
    /*
    * Cost between two objects represents their difference.
    * The bigger the cost, the less similar they are.
    * N rows of cost_matrix represent new objects.
    * T+U columns of cost matrix represent old tracked and untracked objects.
    */
    int N = new_objects.size();
    int T = tracked_objects.size();
    int U = untracked_objects.size();
    cost_matrix.resize(N, T + U);
    cost_matrix.fill(0.0);

    for (int n = 0; n < N; ++n)
    {
        for (int t = 0; t < T; ++t)
        {
            cost_matrix(n, t) = objectCostFunction(new_objects[n], tracked_objects[t].getObject());
        }

        for (int u = 0; u < U; ++u)
        {
            cost_matrix(n, u + T) = objectCostFunction(new_objects[n], untracked_objects[u].getObject());
        }
    }
}

void ROS2Wrapper::calculateRowMinIndices(const MatrixXd &cost_matrix, std::vector<int> &row_min_indices)
{
    /*
    * Vector of row minimal indices keeps the indices of old objects (tracked and untracked)
    * that have the minimum cost related to each of new objects, i.e. row_min_indices[n]
    * keeps the index of old object that has the minimum cost with n-th new object.
    */
    int N, T, U;
    N = cost_matrix.rows();
    T = tracked_objects.size();
    U = untracked_objects.size();

    row_min_indices.assign(N, -1); // Minimum index -1 means no correspondence has been found

    for (int n = 0; n < N; ++n)
    {
        double min_cost = p_min_correspondence_cost;

        for (int t = 0; t < T; ++t)
        {
            if (cost_matrix(n, t) < min_cost)
            {
                min_cost = cost_matrix(n, t);
                row_min_indices[n] = t;
            }
        }

        for (int u = 0; u < U; ++u)
        {
            if (cost_matrix(n, u + T) < min_cost)
            {
                min_cost = cost_matrix(n, u + T);
                row_min_indices[n] = u + T;
            }
        }
    }
}

void ROS2Wrapper::getTrackedParams()
{
    new_detected_objects = objs_msg.objects;
    int N = new_detected_objects.size();
    int T = tracked_objects.size();
    int U = untracked_objects.size();

    if (T + U == 0)
    {
        untracked_objects.assign(new_detected_objects.begin(), new_detected_objects.end());
        return;
    }

    MatrixXd cost_matrix;
    calculateCostMatrix(new_detected_objects, cost_matrix);

    std::vector<int> row_min_indices;
    calculateRowMinIndices(cost_matrix, row_min_indices);

    new_tracked_objects.clear();
    new_untracked_objects.clear();

    for (int n = 0; n < N; ++n)
    {

        if (row_min_indices[n] == -1)
        {
            new_untracked_objects.push_back(new_detected_objects[n]);
        }
        else
        {
            if (row_min_indices[n] >= 0 && row_min_indices[n] < T)
            {
                tracked_objects[row_min_indices[n]].correctState(new_detected_objects[n]);
            }
            else if (row_min_indices[n] >= T)
            { // New detection
                TrackedObject to(untracked_objects[row_min_indices[n] - T]);
                to.correctState(new_detected_objects[n]);
                id_counter++;
                to.setId(id_counter);

                new_tracked_objects.push_back(to);
            }
        }
    }

    tracked_objects.insert(tracked_objects.end(), new_tracked_objects.begin(), new_tracked_objects.end());
    // Remove old untracked objects and save new ones
    untracked_objects.clear();
    untracked_objects.assign(new_untracked_objects.begin(), new_untracked_objects.end());
}

void ROS2Wrapper::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo)
{

    if (!cameraInfo)
    {
        return;
    }

    try
    {
        cameraModel_.fromCameraInfo(*cameraInfo);
    }
    catch (...)
    {
        RCLCPP_INFO(this->get_logger(), "null camera info received");
    }
}


void ROS2Wrapper::switchToRemoteInfCallback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "switchToRemoteInfCallback");
    remote_inference_enable = msg->data;
}


void ROS2Wrapper::remoteDetObjCallback(const object_detection::msg::Objects::SharedPtr remoteDetObjects)
{
    RCLCPP_INFO(get_logger(), "<RemoteInference> Detected Objects Received");
    if ( !remote_inference_enable)
    {
        RCLCPP_INFO(get_logger(), "<RemoteInference> Late Callback Received. Inference mode already switched.");
    }
    else
    {
        std::vector<std::string> labels = objectdetection_.getLabels();
        objs_msg.objects.clear();
        for (uint32_t i = 0; i < remoteDetObjects->objects.size(); i++)
        {
            object_detection::msg::Object r_obj_msg = remoteDetObjects->objects[i];
            // Add object into objs_msg
            if (r_obj_msg.class_id < 0)
                continue;
            obj_msg.class_id = r_obj_msg.class_id;
            obj_msg.confidence = r_obj_msg.confidence;
            obj_msg.label = (static_cast<size_t>(r_obj_msg.class_id) < labels.size() ? labels[r_obj_msg.class_id] : std::string("label #") + std::to_string(r_obj_msg.class_id));
            obj_msg.x = r_obj_msg.x;
            obj_msg.y = r_obj_msg.y;
            obj_msg.height = r_obj_msg.height;
            obj_msg.width = r_obj_msg.width;
            objs_msg.objects.push_back(obj_msg);

            // TODO remove this before merge
            if  (r_obj_msg.class_id != 0)
                RCLCPP_INFO(get_logger(), "<RemoteInference> Label : %s", labels[r_obj_msg.class_id].c_str());


        }


        // ROS publish object list
        if (objs_msg.objects.size() > 0)
        {
            std_msgs::msg::Header header = getHeader();
            objs_msg.header = header;
            pub_objects->publish(objs_msg);
        }

        runTracking();
    }

}

void ROS2Wrapper::depthCallback(const sensor_msgs::msg::Image::SharedPtr depthImage)
{

    if (!depthImage)
    {
        return;
    }

    try
    {
        depthImage_ = cv_bridge::toCvShare(depthImage)->image;
    }
    catch (...)
    {
        RCLCPP_INFO(this->get_logger(), "null depth image received");
    }
}

void ROS2Wrapper::rosCallbackImage(const sensor_msgs::msg::Image::SharedPtr image_msg)
{
    if (!image_msg)
    {
        return;
    }

    if (is_first_image)
    {
        curr_frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
        is_first_image = false;
        // @todo: Validate first input image
        width = (size_t)curr_frame.size().width;
        height = (size_t)curr_frame.size().height;
        step = image_msg->step;
    }
    else
    {
        image = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
        is_new_image = true;
    }

    runObjectDetection();
}

void ROS2Wrapper::publishObjects()
{

    // Publish object
    object_detection::msg::Objects objects_msg;
    for (uint32_t i = 0; i < tracked_objects.size(); i++)
    {
        object_detection::msg::Object object = tracked_objects[i].getObject();
        objects_msg.objects.push_back(object);

        // Draw each bounding box
        std::ostringstream conf;
        conf << ":" << std::fixed << std::setprecision(3) << object.confidence;
        srand(object.id);
        int b = std::rand() % 256;
        int g = std::rand() % 256;
        int r = std::rand() % 256;
        cv::putText(curr_frame,
                    "(" + std::to_string(object.id) + ") " + object.label,
                    cv::Point2f(object.x + 80, object.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5,
                    cv::Scalar(b, g, r), 2);
        cv::rectangle(curr_frame,
                      cv::Point2f(object.x, object.y),
                      cv::Point2f(object.x + object.width, object.y + object.height),
                      cv::Scalar(b, g, r), 4);
    }

    pub_tracked_objects->publish(objects_msg);

    // Publish image_tracked
    std_msgs::msg::Header header = getHeader();
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", curr_frame).toImageMsg();
    img_msg->header = header;
    pub_image_tracked->publish(*img_msg);
}

void ROS2Wrapper::getDistance()
{

    if ((!cameraModel_.initialized()) || depthImage_.cols == 0 || depthImage_.rows == 0)
    {
        return;
    }

    if (depthImage_.cols > 2000 || depthImage_.rows > 2000)
    {
        RCLCPP_WARN(get_logger(), "image too big [%i, %i]", depthImage_.cols, depthImage_.rows);
        return;
    }

    object_detection::msg::Objects objects_msg;
    markers.markers.clear();
    for (uint32_t i = 0; i < tracked_objects.size(); i++)
    {
        object_detection::msg::Object object = tracked_objects[i].getObject();
        objects_msg.objects.push_back(object);

        cv::Rect imageRect(0, 0, depthImage_.cols, depthImage_.rows);

        int pointsCount = 0;
        cv::Point3d points3dSum(0, 0, 0);

        int OFFSET_X = object.width * 0.2;
        int OFFSET_Y = object.height * 0.2;

        for (size_t y = object.y + OFFSET_Y; y < object.y + object.height - OFFSET_Y; y++)
        {
            for (size_t x = object.x + OFFSET_X; x < object.x + object.width - OFFSET_X; x++)
            {
                cv::Point2d uv(x, y);

                if (!imageRect.contains(uv))
                {
                    continue;
                }

                auto point3d = cameraModel_.projectPixelTo3dRay(uv);

                // Read depth in mm and convert to meters
                double depth = (double)depthImage_.at<uint16_t>(uv) / 1000.0;

                // If value is infinite ignore it
                // Realsense cameras are set to max range to 6m by default. Anything above that should be ignored
                if (!std::isfinite(depth) || depth > REALSENSE_MAX_RANGE)
                    continue;

                // Convert to unit vector
                auto d = fmax(0.2, cv::norm(point3d));
                point3d /= d;

                // Scale to actual depth value (in meters)
                point3d *= depth;

                points3dSum += point3d;
                pointsCount++;
            }
        }

        if (pointsCount > 0)
        {

            auto centerOfMass = points3dSum / pointsCount;

            cv::Point2d centerPixel(object.x + object.width / 2,
                                    object.y + object.height / 2);
            auto objectRay = cameraModel_.projectPixelTo3dRay(centerPixel);

            double bearing = atan2(objectRay.z, -objectRay.x);
            double distance = cv::norm(centerOfMass) - 0.1;

            tf2::Vector3 objectVector(distance * sin(bearing),
                                      distance * cos(bearing), 0);

            std_msgs::msg::Header header = getHeader();
            visualization_msgs::msg::Marker marker;
            visualization_msgs::msg::Marker marker_3d;

            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.header = header;
            marker.ns = "object_cube";
            marker.id = object.id;
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = -objectVector.y();
            marker.pose.position.y = 0.2;
            marker.pose.position.z = objectVector.x();
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.text = object.label;

            markers.markers.push_back(marker);

            marker_3d.action = visualization_msgs::msg::Marker::ADD;
            marker_3d.type = visualization_msgs::msg::Marker::CUBE;
            marker_3d.header = header;
            marker_3d.ns = "object_label";
            marker_3d.id = object.id;
            marker_3d.pose.orientation.w = 1.0;
            marker_3d.pose.position.x = -objectVector.y();
            marker_3d.pose.position.y = 0;
            marker_3d.pose.position.z = objectVector.x();
            marker_3d.scale.x = 0.1;
            marker_3d.scale.y = 0.1;
            marker_3d.scale.z = 0.1;

            marker_3d.color.a = 1.0;
            marker_3d.color.g = 1.0;
            marker_3d.text = object.label;

            markers.markers.push_back(marker_3d);
        }
    }

}

void ROS2Wrapper::runObjectDetection()
{
    // Capture frame
    if (image.empty())
    {
        RCLCPP_WARN(get_logger(), "No data received in CameraTopic instance");
        return;
    }

    if (!cameraModel_.initialized() || cameraModel_.tfFrame().empty())
        return;

    // Current image is old
    if (!is_new_image)
        return;

    next_frame = image;

    if (remote_inference_enable)
    {  
        RCLCPP_INFO(get_logger(), "<RemoteInference> Sending Image");

        sensor_msgs::msg::Image::SharedPtr img;
        img = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
        img->width = width;
        img->height = height;
        img->is_bigendian = false;
        img->step = step;
        img->header.frame_id = "color_optical_frame_id";

        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
        int64 ns = tp.time_since_epoch().count();
        img->header.stamp.sec = ns / 1000000000;
        img->header.stamp.nanosec = ns % 1000000000;

        pub_remote_infer.publish(img);
    }
    else
    {
        objectdetection_.getBoundingBoxes(next_frame);
        setBoundingBoxParameters();
    }

    // Final point:
    // in the truly Async mode we swap the NEXT and CURRENT requests for the next iteration
    curr_frame = next_frame;
    next_frame = cv::Mat();
    is_new_image = false;
}

void ROS2Wrapper::setBoundingBoxParameters()
{
    std::vector<DetectedObjectBoxes> det_objects = objectdetection_.getDetectedObjects();

    objs_msg.objects.clear();
    for (uint32_t i = 0; i < det_objects.size(); i++)
    {
        DetectedObjectBoxes obj = det_objects[i];
        // Add object into objs_msg
        obj_msg.class_id = obj.class_id;
        obj_msg.confidence = obj.confidence;
        obj_msg.label = obj.label;
        obj_msg.x = obj.x;
        obj_msg.y = obj.y;
        obj_msg.height = obj.height;
        obj_msg.width = obj.width;
        objs_msg.objects.push_back(obj_msg);

        std_msgs::msg::Header header = getHeader();
        // ROS publish object list
        if (objs_msg.objects.size() > 0)
        {
            objs_msg.header = header;
            pub_objects->publish(objs_msg);
        }
    }
    runTracking();
}

std_msgs::msg::Header ROS2Wrapper::getHeader()
{
    std_msgs::msg::Header header;
    header.frame_id = cameraModel_.tfFrame();

    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    int64 ns = tp.time_since_epoch().count();
    header.stamp.sec = ns / 1000000000;
    header.stamp.nanosec = ns % 1000000000;
    return header;
}
