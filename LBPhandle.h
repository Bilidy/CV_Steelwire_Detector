#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define ALPHA  5
using namespace cv;
int getHopCount(uchar i);
void lbp36table(uchar* _36table);
void lbp58table(uchar* table);
void lbp9table(uchar* _58table,uchar* _36table,uchar* _9table);
void LBP(Mat& _src, Mat& _dst,uchar* table);