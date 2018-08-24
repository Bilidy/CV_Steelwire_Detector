#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define MODEL_NUM 5
#define HISTO_SIZE 58//通用旋转不变LBP
using namespace cv;

typedef struct histogram
{
	unsigned int *histo;
	float *normalHisto;
	float weight;
	float normalWeight;
}Histogram;//直方图结构体

class RegionBlock
{

private: 
	Histogram histogram[MODEL_NUM];
public:

	Point leftTopPoint;
	Point rightBottomPoint;
	float weight_initial;
	unsigned int Width;
	unsigned int Hight;
	uchar bgThresholdLevel;
	Histogram* getHisogram();
	void setWeight(unsigned int level,float weight);
	RegionBlock();
	RegionBlock(RegionBlock &_regionBlock);
	RegionBlock(Point _leftTopPoint,Point _rightBottomPoint);
	~RegionBlock(void);
};

