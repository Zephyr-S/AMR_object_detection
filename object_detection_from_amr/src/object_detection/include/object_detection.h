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

#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <boost/bind.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <inference_engine.hpp>
#include <utils/common.hpp>
#include <utils/ocv_common.hpp>

#include <models/detection_model_ssd.h>
#include <pipelines/async_pipeline.h>
#include <utils/args_helper.hpp>
#include <utils/slog.hpp>
#include <utils/images_capture.h>
#include <utils/default_flags.hpp>
#include <utils/performance_metrics.hpp>
#include <unordered_map>
#include <pipelines/metadata.h>

using namespace InferenceEngine;

class DetectedObjectBoxes
{
public:
    int class_id;
    std::string label;
    float confidence;
    double score_threshold;
    float x;
    float y;
    float height;
    float width;
};

class ObjectDetection
{

public:
    ObjectDetection();

    ~ObjectDetection();

    void initInferenceEngine(const std::string &device, const std::string &model_path);

    void getBoundingBoxes(const cv::Mat &curr_frame);

    std::vector<DetectedObjectBoxes> getDetectedObjects() const { return det_objects; }
    std::vector<std::string> getLabels() const { return labels; }

private:
    void interpret_results(DetectionResult& result);

    int64_t frameNum = -1;
    uint32_t framesProcessed = 0;
 
    std::unique_ptr<ModelBase> model;
    std::unique_ptr<ResultBase> result;
    std::unique_ptr<AsyncPipeline> pipeline;
    InferenceEngine::Core core;

    //TODO: use this for stats
    OutputTransform outputTransform;
    PerformanceMetrics metrics;
    PerformanceMetrics renderMetrics;
    // **************************
    // IE VARIABLES
    // **************************
    std::string model_path;
    std::string device;
    double score_threshold;
    bool bool_auto_resize;
    bool bool_pc;

    Core ie;

    // **************************
    // Application Variables
    // **************************
    std::string imageInputName;
    std::string imageInfoInputName;
    std::string outputName;

    SizeVector outputDims;
    int maxProposalCount;
    int objectSize;

    std::vector<std::string> labels;
    std::vector<DetectedObjectBoxes> det_objects;
};
