#define OPENCV

#include "yolo_head.h"
#include "yolo_v2_class.hpp"
using namespace std;
using namespace cv;
#define GPU

int main(void) {
  Mat frame = imread("../data/dog.jpg");
  yoloDetector(frame);
  imshow("frame", frame);
  waitKey(0);
  return 0;
}
