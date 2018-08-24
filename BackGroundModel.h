#pragma once
#include "RegionBlock.h"	//LBPƫ����
#define ALPHA_WEIGHT 0.06	//0.01<ALPHA_WEIGHT<0.05
#define ALPHA_HIST 0.06		//0.01<ALPHA_HIST<0.05
#define BGT 0.9				//0.7<Background Threshold<0.9�ж�Ϊ��������ֵ
#define TB 0.8				//0.5<TB<0.7ֱ��ͼ��С������ֵ��ֵԽ��ǰ��Խ���׳��֣��������Ҳ����Ӧ���
#define INC_STEP 10			//��ƵͶƱ����
#define THRHD 1				//��ֵ����
class BackGroundModel
{
private:
	Mat initFrame;
	Mat LBPmat;

	Mat frontMaskMat;
	Mat frontMat;
	Mat TempLBPmat;

	uchar* table;

	unsigned int backGroundWidth;		//�������=Image.cols
	unsigned int backGroundHight;		//�����߶�=Image.rows

	unsigned int blockWidth;			//block�Ŀ��
	unsigned int blockHight;			//block�ĸ߶�

	unsigned int Hshift;				//ˮƽ�����λ����=blockHight/2
	unsigned int Vshift;				//��ֱ�����λ����=blockWidth/2

	unsigned int BblockHnum;			//��bolckˮƽ���������=(backGroundWidth/Hshift)-1
	unsigned int BblockVnum;			//��block��ֱ���������=(backGroundWidth/Vshift)-1

	unsigned int SblockHnum;			//Сbolckˮƽ���������=(backGroundWidth/Hshift)
	unsigned int SblockVnum;			//Сblock��ֱ���������=(backGroundWidth/Vshift)

	unsigned int BblockNum;				//��bolck������=(BblockHnum*BblockVnum)
	unsigned int SblockNum;				//Сblock������=(SblockHnum*SblockVnum)

	uchar **Bblock;						//��block�ı�־��ά����
	RegionBlock **BregionBlocks;		//BregionBlocks��ά����ָ��
	float*** histoMat;					//ֱ��ͼ����Ϊ���������Ч�ʣ��ڱ���ģ�͸��µ�ʱ��ʹ����ά
										//����洢��ͼƬ�������LBPֱ��ͼ


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

