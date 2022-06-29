#include "yolo.h"
#include "yolo_v2_class.hpp"


void DrawBoxes(cv::Mat &frame, std::vector<std::string> classes,
               int classId, float conf,
               int left, int top, int right, int bottom)
{

    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
                  cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}


void Drawer(cv::Mat &frame, std::vector<bbox_t> outs, std::vector<std::string> classes)
{
    
    for (int i = 0; i < outs.size(); i++)
    {
        DrawBoxes(frame, classes, outs[i].obj_id, outs[i].prob, outs[i].x, outs[i].y,
                  outs[i].x + outs[i].w, outs[i].y + outs[i].h);
    }
}

void yoloDetector(cv::Mat frame,
                  std::vector<std::string> &classesVec,
                  Detector &detector)
{
    //    std::string classes = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/coco.names";
    //    std::string modelConfig = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/yolov4.cfg";
    //    std::string modelWeights = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/yolov4.weights";

    
    std::shared_ptr<image_t> detImg = detector.mat_to_image_resize(frame);
    
    std::vector<bbox_t> outs = detector.detect_resized(*detImg, frame.cols, frame.rows, 0.25);
    
    Drawer(frame, outs, classesVec);
    //    imwrite("../data/result.jpg", frame);
}
