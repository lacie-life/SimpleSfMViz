#include "yolo.h"
#include "yolo_v2_class.hpp"

//画出检测框和相关信息
void DrawBoxes(cv::Mat &frame, std::vector<std::string> classes,
               int classId, float conf,
               int left, int top, int right, int bottom)
{
    //画检测框
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    //该检测框对应的类别和置信度
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    //将标签显示在检测框顶部
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
                  cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}

//画出检测结果
void Drawer(cv::Mat &frame, std::vector<bbox_t> outs, std::vector<std::string> classes)
{
    //获取所有最佳检测框信息
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

    //图像
    std::shared_ptr<image_t> detImg = detector.mat_to_image_resize(frame);
    //前向预测
    std::vector<bbox_t> outs = detector.detect_resized(*detImg, frame.cols, frame.rows, 0.25);
    //画图
    Drawer(frame, outs, classesVec);
    //    imwrite("../data/result.jpg", frame);
}
