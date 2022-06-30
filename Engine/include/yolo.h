#define OPENCV
#define GPU
#include "yolo_v2_class.hpp"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

void DrawBoxes(cv::Mat &frame, std::vector<std::string> classes,
               int classId, float conf,
               int left, int top, int right, int bottom);

void Drawer(cv::Mat &frame,
            std::vector<bbox_t> outs,
            std::vector<std::string> classes);

void yoloDetector(cv::Mat frame,
                  std::vector<std::string> &classesVec,
                  Detector &detector);
