#include "stdafx.h" 
#include "RegionBlock.h"

RegionBlock::RegionBlock()
{
}
/*
RegionBlock�Ĺ��캯��
*/
RegionBlock::RegionBlock(Point _leftTopPoint,Point _rightBottomPoint)
{
	weight_initial=1.0/MODEL_NUM;//���ó�ʼȨֵ
	leftTopPoint=_leftTopPoint;
	rightBottomPoint=_rightBottomPoint;
	Width=_rightBottomPoint.x-_leftTopPoint.x;
	Hight=_rightBottomPoint.y-_leftTopPoint.y;
	for(int i=0;i<MODEL_NUM;++i)
	{
		histogram[i].histo=new unsigned int[HISTO_SIZE];
		histogram[i].normalHisto=new float[HISTO_SIZE];
		memset(histogram[i].histo,0,sizeof(unsigned int)*HISTO_SIZE);
		memset(histogram[i].normalHisto,0,sizeof(float)*HISTO_SIZE);
		histogram[i].weight=weight_initial;
	}

}
/*
RegionBlock�Ŀ������캯��
*/
RegionBlock::RegionBlock(RegionBlock &_regionBlock)
{
	weight_initial=_regionBlock.weight_initial;//���ó�ʼȨֵ
	leftTopPoint=_regionBlock.leftTopPoint;
	rightBottomPoint=_regionBlock.rightBottomPoint;
	Width=_regionBlock.rightBottomPoint.x-_regionBlock.leftTopPoint.x;
	Hight=_regionBlock.rightBottomPoint.y-_regionBlock.leftTopPoint.y;
	for(int i=0;i<MODEL_NUM;++i)
	{
		histogram[i].histo=new unsigned int[HISTO_SIZE];
		histogram[i].normalHisto=new float[HISTO_SIZE];
		for(int j=0;j<HISTO_SIZE;++j)
		{
			
			histogram[i].histo[j]=_regionBlock.histogram[i].histo[j];
			histogram[i].normalHisto[j]=_regionBlock.histogram[i].normalHisto[j];
		}
		histogram[i].weight=weight_initial;
	}

}
Histogram* RegionBlock::getHisogram()
{
	return histogram;
}
/*
��RegionBlock��level��ε�Ȩֵ��������
*/
void RegionBlock::setWeight(unsigned int level,float weight)
{
	if(level>=MODEL_NUM)
		return;
	histogram[level].normalWeight=weight;
}

RegionBlock::~RegionBlock(void)
{
	for(int i=0;i<MODEL_NUM;++i)
	{
		delete[] histogram[i].histo;
		delete[] histogram[i].normalHisto;
	}
}
