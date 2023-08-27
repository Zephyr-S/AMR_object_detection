/*
 INTEL CONFIDENTIAL
 Copyright 2021 Intel Corporation
 This software and the related documents are Intel copyrighted materials, and your use of them is governed by the
 express license under which they were provided to you (License). Unless the License provides otherwise, you may not
 use, modify, copy, publish, distribute, disclose or transmit this software or the related documents without Intel's
 prior written permission.
 This software and the related documents are provided as is, with no express or implied warranties, other than those
 that are expressly stated in the License.
*/


#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/bind.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <object_detection/msg/object.hpp>
#include <object_detection/msg/objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>

#include "tracked_object.h"
#include "object_detection.h"

using namespace Eigen;

class ROS2Wrapper : public rclcpp::Node
{

public:
    ROS2Wrapper();

    ~ROS2Wrapper();

private:
    void setBoundingBoxParameters();

    void runTracking();

    void runObjectDetection();

    void getParameters();

    void initROSInterface();

    void getDistance();

    void setParameters();

    void rosCallbackImage(const sensor_msgs::msg::Image::SharedPtr image_msg);

    void remoteDetObjCallback(const object_detection::msg::Objects::SharedPtr remoteDetObjects);

    void switchToRemoteInfCallback(const std_msgs::msg::UInt32::SharedPtr data);

    void getTrackedParams();

    void updateTrackedObjects();

    void publishObjects();

    double objectCostFunction(const object_detection::msg::Object &new_object, const object_detection::msg::Object &old_object);

    void calculateCostMatrix(const std::vector<object_detection::msg::Object> &new_objects, MatrixXd &cost_matrix);

    void calculateRowMinIndices(const MatrixXd &cost_matrix, std::vector<int> &row_min_indices);

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr depthImage);

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo);

    std_msgs::msg::Header getHeader();

    void updateMarkers();

    // **************************
    // ROS INTERFACE
    // **************************

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_rgb;
    rclcpp::Publisher<object_detection::msg::Objects>::SharedPtr pub_objects;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_tracked;
    rclcpp::Publisher<object_detection::msg::Objects>::SharedPtr pub_tracked_objects;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_rects;

    //pose parameters

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscriber_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoSubscriber_;

    rclcpp::Subscription<object_detection::msg::Objects>::SharedPtr sub_remote_det_objects;

    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_switch_inference_type;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;

    image_transport::Publisher pub_remote_infer;
    image_geometry::PinholeCameraModel cameraModel_;

    visualization_msgs::msg::MarkerArray markers;

    // Create objects
    object_detection::msg::Objects objs_msg;
    object_detection::msg::Object obj_msg;

    std::string topic_image_input;
    std::string topic_TrackedImage_output;
    std::string topic_objects_output;
    std::string topic_TrackedObjects_output;
    std::string topic_depth_image;
    std::string topic_camera_info;

    std::string topic_image_output;
    std::string topic_image_remote;
    std::string topic_remote_obj_detection;
    std::string topic_switch_to_remote_inf;

    // **************************
    // IE VARIABLES
    // **************************
    std::string model_path;
    std::string device;
    bool remote_inference_enable;

    // **************************
    // Application Variables
    // **************************

    cv::Mat image;
    cv::Mat curr_frame;
    cv::Mat next_frame;
    cv::Mat depthImage_;

    bool is_new_image;
    bool is_first_image;

    size_t width;
    size_t height;
    size_t step;

    // **************************
    // Tracker Variables
    // **************************

    std::vector<object_detection::msg::Object> new_detected_objects;
    std::vector<TrackedObject> tracked_objects;
    std::vector<TrackedObject> untracked_objects;
    std::vector<TrackedObject> new_tracked_objects;
    std::vector<TrackedObject> new_untracked_objects;

    int id_counter;

    double p_min_correspondence_cost;
    double p_loop_rate;
    double p_tracking_duration;

    int p_fade_counter_size;
    double p_sampling_time;
    double p_process_variance;
    double p_process_rate_variance;
    double p_measurement_variance;
    
    ObjectDetection objectdetection_;
};
