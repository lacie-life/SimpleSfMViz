#define OPENCV
#include "yolo_v2_class.hpp"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;
#define GPU

void DrawBoxes(Mat &frame, vector<string> classes, int classId, float conf, int left, int top, int right, int bottom);

void Drawer(Mat &frame, vector<bbox_t> outs, vector<string> classes);

void yoloDetector(Mat frame);