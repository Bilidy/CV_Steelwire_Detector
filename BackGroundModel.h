#pragma once
#include "RegionBlock.h"	//LBP偏移量
#define ALPHA_WEIGHT 0.06	//0.01<ALPHA_WEIGHT<0.05
#define ALPHA_HIST 0.06		//0.01<ALPHA_HIST<0.05
#define BGT 0.9				//0.7<Background Threshold<0.9判定为背景的阈值
#define TB 0.8				//0.5<TB<0.7直方图最小交集阈值。值越大前景越容易出现，但是噪点也会相应变多
#define INC_STEP 10			//视频投票步长
#define THRHD 1				//阈值阀门
class BackGroundModel
{
private:
	Mat initFrame;
	Mat LBPmat;

	Mat frontMaskMat;
	Mat frontMat;
	Mat TempLBPmat;

	uchar* table;

	unsigned int backGroundWidth;		//背景宽度=Image.cols
	unsigned int backGroundHight;		//背景高度=Image.rows

	unsigned int blockWidth;			//block的宽度
	unsigned int blockHight;			//block的高度

	unsigned int Hshift;				//水平方向的位移量=blockHight/2
	unsigned int Vshift;				//竖直方向的位移量=blockWidth/2

	unsigned int BblockHnum;			//大bolck水平方向的数量=(backGroundWidth/Hshift)-1
	unsigned int BblockVnum;			//大block竖直方向的数量=(backGroundWidth/Vshift)-1

	unsigned int SblockHnum;			//小bolck水平方向的数量=(backGroundWidth/Hshift)
	unsigned int SblockVnum;			//小block竖直方向的数量=(backGroundWidth/Vshift)

	unsigned int BblockNum;				//大bolck的数量=(BblockHnum*BblockVnum)
	unsigned int SblockNum;				//小block的数量=(SblockHnum*SblockVnum)

	uchar **Bblock;						//大block的标志二维数组
	RegionBlock **BregionBlocks;		//BregionBlocks二维数组指针
	float*** histoMat;					//直方图矩阵，为了提高运算效率，在背景模型更新的时候使用三维
										//矩阵存储新图片计算出的LBP直方图


	float alpha;
	float alpha_weight;
	float alpha_hist;
	float bgt;
	float tb;
	bool histoIntersection(RegionBlock& regionBlock,unsigned int index);
public:
	void setInitframe(Mat frame);
	void setLBPmod(uchar* table);
	void calculateLBP();
	void calculateInitHisto();
	void BackGroundModel::frontDetect(Mat newMat,Mat&outputMat);
	
	void calculateBGHisto(RegionBlock& regionBlock,unsigned int index);
	void BackGroundModel::release();
	void BackGroundModel::updateBackGroundModel(Mat& newMat);
	BackGroundModel(Mat&frame,unsigned int blockWidth,unsigned int blockHight);
	~BackGroundModel(void);
};

