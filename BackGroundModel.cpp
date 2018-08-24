#include "stdafx.h" 
#include "BackGroundModel.h"
#include "LBPhandle.h"
void normalWeight(RegionBlock& regionBlock);
void sortModelsByWeight(RegionBlock& regionBlock);
void set58histo(uchar value,float*histo,int&counter);
void set9histo(uchar value,float*histo,int&counter);
void normalHisto(float * normalHisto,unsigned int sum);
void normalHisto(float * normalHisto);
BackGroundModel::BackGroundModel(Mat&frame,unsigned int _blockWidth,unsigned int _blockHight)
{
	alpha=ALPHA;
	alpha_weight=ALPHA_WEIGHT;
	alpha_hist=ALPHA_HIST;
	bgt=BGT;
	tb=TB;
	frontMaskMat=Mat::zeros(frame.rows,frame.cols,CV_8U);
	frontMat=Mat::zeros(frame.rows,frame.cols,CV_8U);
	if(_blockWidth==0||_blockHight==0)
	{
		blockWidth=10;
		blockHight=10;
	}else
	{
		blockWidth=_blockWidth;
		blockHight=_blockHight;
	}

	Vshift=blockHight/2;
	Hshift=blockWidth/2;

	backGroundWidth=frame.cols;
	backGroundHight=frame.rows;

	BblockHnum=(backGroundWidth/Hshift)-1;
	BblockVnum=(backGroundHight/Vshift)-1;

	SblockHnum=(backGroundWidth/Hshift);
	SblockVnum=(backGroundHight/Vshift);

	BblockNum=(BblockHnum*BblockVnum);
	SblockNum=(SblockHnum*SblockVnum);

	Bblock=(uchar**)malloc(sizeof(uchar*)*BblockVnum);
	for(int i=0;i<BblockVnum;++i)
	{
		Bblock[i]=(uchar*)malloc(sizeof(uchar)*BblockHnum);
		memset(Bblock[i],0,BblockHnum);
	}
	//为BregionBlocks分配空间
	BregionBlocks=(RegionBlock**)malloc(sizeof(RegionBlock*)*BblockVnum);
	for(int i=0;i<BblockVnum;++i)
	{
		BregionBlocks[i]=(RegionBlock*)malloc(sizeof(RegionBlock)*BblockHnum);
		for(int j=0;j<BblockHnum;++j)
		{
			Point leftTopPoint(j*Hshift,i*Vshift);
			Point rightBottomPoint(j*Hshift+blockWidth,i*Vshift+blockHight);
			RegionBlock *block=new RegionBlock(leftTopPoint,rightBottomPoint);
			BregionBlocks[i][j]=*block;
			//delete block;
		}
	}
	histoMat=(float***)malloc(sizeof(float**)*BblockVnum);
	for(int i=0;i<BblockVnum;++i)
	{
		histoMat[i]=(float**)malloc(sizeof(float*)*BblockHnum);
		for(int j=0;j<BblockHnum;++j)
		{
			float *histo=(float*)malloc(sizeof(float)*HISTO_SIZE);
			memset(histo,0,HISTO_SIZE);
			histoMat[i][j]=histo;
		}
	}
}
///
//将当配置背景更新帧为当前帧
//
///
void BackGroundModel::setInitframe(Mat frame)
{
	initFrame=frame;
};
///
//设置通过设置不同的table来设置不同的工作模式
///
void BackGroundModel::setLBPmod(uchar* _table)
{
	table=(uchar*)malloc(sizeof(uchar)*256);
	memset(table,0,256);
	for(int i=0;i<256;++i)
	{
		table[i]=_table[i];
	}
};
///
//计算帧的LBP
///
void BackGroundModel::calculateLBP()
{
	LBP(initFrame,LBPmat,table);
};
///
/*经过计算后使用得到的LBP初始化背景模型，包括：
对每个blockregin中MODEL_NUM个层次的模型赋予初始权值
对每个blockregin中MODEL_NUM个层次的模型赋予初始LBP直方图
将每个blockregin的权值归一化，并且由高到低排序
将每个blockregin中MODEL_NUM个层次的模型LBP直方图归一化
分割模型层次，根据划分阈值BGT将多层次模型划分为前景和背景*/
///
void BackGroundModel::calculateInitHisto()
{
	for(int i=0;i<BblockVnum;++i)
	{
		for(int j=0;j<BblockHnum;++j)
		{
			calculateBGHisto(BregionBlocks[i][j],0);
			BregionBlocks[i][j].setWeight(0,1.);//初始化权值
			for(int modelnum=1;modelnum<MODEL_NUM;++modelnum)
			{
				BregionBlocks[i][j].setWeight(modelnum,1.);//初始化权值
				for(int histosize=0;histosize<HISTO_SIZE;++histosize)
				{
					BregionBlocks[i][j].getHisogram()[modelnum].histo[histosize]=BregionBlocks[i][j].getHisogram()[0].histo[histosize];
					BregionBlocks[i][j].getHisogram()[modelnum].normalHisto[histosize]=BregionBlocks[i][j].getHisogram()[0].normalHisto[histosize];
				}
			}
			normalWeight(BregionBlocks[i][j]);//归一化权值
			sortModelsByWeight(BregionBlocks[i][j]);
			int num=0;
			float weightsum=0.0;
			//计算背景分割模型层次
			while(weightsum<BGT&&num<MODEL_NUM)
			{
				weightsum+=BregionBlocks[i][j].getHisogram()[num].normalWeight;
				num++;
			}
			BregionBlocks[i][j].bgThresholdLevel=num-1;
		}
	}
};
///
//calculateHisto(RegionBlock& regionBlock,unsigned int index)
//计算RegionBlock的某个level的直方图
//并且归一化这个level的直方图
///
void BackGroundModel::calculateBGHisto(RegionBlock& regionBlock,unsigned int level)
{
	memset(regionBlock.getHisogram()[level].histo,0,sizeof(unsigned int)*HISTO_SIZE);
	memset(regionBlock.getHisogram()[level].normalHisto,0,sizeof(float)*HISTO_SIZE);
	int counter=0;
	for(int i=regionBlock.leftTopPoint.y;i<regionBlock.leftTopPoint.y+regionBlock.Hight;++i)
	{
		for(int j=regionBlock.leftTopPoint.x;j<regionBlock.leftTopPoint.x+regionBlock.Width;++j)
		{
			uchar value=LBPmat.at<uchar>(i,j);
			set58histo(value,regionBlock.getHisogram()[level].normalHisto,counter);
		}
	}
	//归一化直方图
	normalHisto(regionBlock.getHisogram()[level].normalHisto,counter);
}
///
//normalWeight(RegionBlock& regionBlock)
//
//归一化这个RegionBlock的权值
///
void normalWeight(RegionBlock& regionBlock)
{
	float weightcounter=0.0;
	for(int i=0;i<MODEL_NUM;++i)
	{
		weightcounter+=regionBlock.getHisogram()[i].normalWeight;
	}
	for(int i=0;i<MODEL_NUM;++i)
	{
		regionBlock.getHisogram()[i].normalWeight=regionBlock.getHisogram()[i].normalWeight/weightcounter;
	}
}
///
//对RegionBlock的权值进行排序，由高到低。
//
///
void sortModelsByWeight(RegionBlock& regionBlock)
{

	Histogram	tempHistogram;
	unsigned int * tempHistoPtr;
	int postion=0;
	for(int i=0;i<MODEL_NUM;++i)
	{
		float max=0.0;
		for(int j=i;j<MODEL_NUM;++j)
		{

			if(regionBlock.getHisogram()[j].normalWeight>max)
			{
				max=regionBlock.getHisogram()[j].normalWeight;
				postion=j;
				//histoPtr=regionBlock.getHisogram()[j].histo;
				//normalHistoPtr=regionBlock.getHisogram()[j].normalHisto;

			}
		}
		tempHistogram=regionBlock.getHisogram()[i];
		regionBlock.getHisogram()[i]=regionBlock.getHisogram()[postion];
		regionBlock.getHisogram()[postion]=tempHistogram;
		postion=i;
	}
}
///
/*
归一化直方图。有两种方式，
一种是知道直方图每个数据柱的综合，将直接计算出结果。
另一种是不知道总和，需要临时统计。
第一种方法使用函数normalHisto(float * normalHisto,unsigned int sum)
第二种方法使用函数normalHisto(float * normalHisto)
*/
///
void normalHisto(float * normalHisto)
{
	float sum=0.0;
	for(int histosize=0;histosize<HISTO_SIZE;++histosize)
	{
		sum+=normalHisto[histosize];
	}
	for(int histosize=0;histosize<HISTO_SIZE;++histosize)
	{
		normalHisto[histosize]=(float)normalHisto[histosize]/sum;
	}
};
void normalHisto(float * normalHisto,unsigned int sum)
{
	for(int histosize=0;histosize<HISTO_SIZE;++histosize)
	{
		normalHisto[histosize]=(float)normalHisto[histosize]/sum;
	}
};
///
//对背景模型进行更新
//
///
void BackGroundModel::updateBackGroundModel(Mat& newMat)
{
	for(int i=0;i<BblockVnum;++i)
	{
		for(int j=0;j<BblockHnum;++j)
		{
			int level=0;
			while(((!histoIntersection(BregionBlocks[i][j],level))&&(level<MODEL_NUM)))
			{
				level++;
			}
			if(MODEL_NUM==level)//说明所有模型都没有得到匹配
			{
				for(int index=0;index<HISTO_SIZE;++index)
				{
					BregionBlocks[i][j].getHisogram()[MODEL_NUM-1].normalHisto[index]=histoMat[i][j][index];
				}
				//The new histogram is given a low initial weight.
				//In our experiments, a value of 0.01 was used.
				BregionBlocks[i][j].getHisogram()[MODEL_NUM-1].normalWeight=0.01;
				normalWeight(BregionBlocks[i][j]);
			}
			else //有匹配到模型
			{
				//The best matching model histogram is adapted with 
				//the new data by updating its bins as follows:
				//mk = αb*h + (1 − αb)*mk 
				//where αb is a user-settable learning rate,we define the rate as ALPHA_HIST
				for(int index=0;index<HISTO_SIZE;++index)
				{
					float mk=BregionBlocks[i][j].getHisogram()[level].normalHisto[index];
					float h=histoMat[i][j][index];
					BregionBlocks[i][j].getHisogram()[level].normalHisto[index]=ALPHA_HIST*h+(1-ALPHA_HIST)*mk;
				}
				///直方图归一化
				/*for(int histosize=0;histosize<HISTO_SIZE;++histosize)
				{
					BregionBlocks[i][j].getHisogram()[level].normalHisto[histosize]=BregionBlocks[i][j].getHisogram()[level].normalHisto[histosize]/counter;
				}*/
				normalHisto(BregionBlocks[i][j].getHisogram()[level].normalHisto);

				//The weight of the kth model histogram is denoted by ωk.
				//ωk = αwMk + (1 − αw)ωk,
				//where αw is another user-settable learning rate 
				//and Mk is 1 for the best matching histogram and 0 for the others. 
				float bestMatchingWeight=ALPHA_WEIGHT*1+(1-ALPHA_WEIGHT)*BregionBlocks[i][j].getHisogram()[level].normalWeight;
				for(int nomatchlevel=0;nomatchlevel<MODEL_NUM;++nomatchlevel)
				{
					float noMatchingWeight=ALPHA_WEIGHT*0+(1-ALPHA_WEIGHT)*BregionBlocks[i][j].getHisogram()[nomatchlevel].normalWeight;
					BregionBlocks[i][j].getHisogram()[nomatchlevel].normalWeight=noMatchingWeight;
				}
				BregionBlocks[i][j].getHisogram()[level].normalWeight=bestMatchingWeight;
				normalWeight(BregionBlocks[i][j]);
			}
			sortModelsByWeight(BregionBlocks[i][j]);

			int modelnum=0;
			float weightsum=0.0;
			//计算背景分割模型层次
			while(weightsum<BGT&&modelnum<MODEL_NUM)
			{
				weightsum+=BregionBlocks[i][j].getHisogram()[modelnum].normalWeight;
				modelnum++;
			}
			BregionBlocks[i][j].bgThresholdLevel=modelnum-1;
		}
	}
	//需要对比相应位置直方图
}
///
//检测前景
///
void BackGroundModel::frontDetect(Mat newMat,Mat &outputMat)
{
	frontMaskMat=0;
	frontMat=0;

	LBP(newMat,TempLBPmat,table);//我们会得到更新图的LBP
	//imshow("LBP",TempLBPmat);

	for(int i=0;i<BblockVnum;++i)
	{
		for(int j=0;j<BblockHnum;++j)
		{
			/*
			组装直方图，需要遍历当前Block的所有的像素。
			当前Block所占据的像素范围为(j*Hshift,i*Vshift)
			到(j*Hshift+blockWidth,i*Vshift+blockHight)
			在此范围内的像素只都会由set58histo()函数统计
			到,histoMat[i][j]中，形成对应block坐标的LBP直方图
			随后将这个直方图归一化，使用normalHisto()函数
			*/
			int counter=0;
			for(int x=j*Hshift;x<j*Hshift+blockWidth;++x)
			{
				for(int y=i*Vshift;y<i*Vshift+blockHight;++y)
				{
					uchar value=TempLBPmat.at<uchar>(y,x); 
					set58histo(value,histoMat[i][j],counter);
				}
			}
			//对直方图归一化
			normalHisto(histoMat[i][j],counter);
			/*
			对应坐标的MODEL_NUM个背景直方图由其权值从高到低和用于
			更新背景的新图的直方图做比较。
			其中index用来标识匹配到的位置编号（背景直方图的权值排序
			可以保证得到匹配的背景模型的权值是最大的）
			*/
			int index=0;
			while(!histoIntersection(BregionBlocks[i][j],index)&&index<MODEL_NUM)
			{
				index++;
			}
			/*
			背景的确定方法如下:将 LBP统一模式直方
			图按照权值进行降序排序,并在 K 个降序排列
			好的直方图中选取满足下式的最少的前 B 个直
			方图来表征背景(B<K):
				w0+…+wB-1>BGT,BGT∈[0,1]
			式中:BGT为用户设定的一个阈值,其值与K紧密相关
			BregionBlocks[i][j].bgThresholdLevel指对应的block背景
			模型中前0到bgThresholdLevel的模型权值和大于BGT。
			对于上一步匹配到的index，和bgThresholdLevel做对比，
			如果index落在bgThresholdLevel之后则认为是前景，否则为背景。
			*/
			if(BregionBlocks[i][j].bgThresholdLevel<index)//此为前景
			{
				/*
				通过对匹配到的前景block内各个坐标进行投票，来记录被
				认为是前景次数最多的坐标位置，率除掉噪点带来的误检测
				*/
				
				for(int x=j*Hshift;x<j*Hshift+blockWidth;++x)
				{
					for(int y=i*Vshift;y<i*Vshift+blockHight;++y)
					{
						frontMaskMat.at<uchar>(y,x)=frontMaskMat.at<uchar>(y,x)+INC_STEP;
					}
				}
				for(int x=j*Hshift;x<j*Hshift+blockWidth;++x)
				{
					for(int y=i*Vshift;y<i*Vshift+blockHight;++y)
					{
						if(frontMaskMat.at<uchar>(y,x)>THRHD*INC_STEP)
						{
							frontMat.at<uchar>(y,x)=newMat.at<uchar>(y,x)*1;
						}
					}
				}
			}
		}
	}
	outputMat=frontMat;
	imshow("未经过误检测过滤",frontMaskMat>0);
	imshow("Mask", frontMaskMat);
	imshow("经过误检测过滤", frontMaskMat>INC_STEP*THRHD);
}
bool BackGroundModel::histoIntersection(RegionBlock& regionBlock,unsigned int index)
{
	int j=regionBlock.leftTopPoint.x/Hshift;
	int i=regionBlock.leftTopPoint.y/Vshift;
	float sum=0.0;

	if(index>=MODEL_NUM)
	{
		return false;
	}
	for(int histoIndex=0;histoIndex<HISTO_SIZE;++histoIndex)
	{
		sum+=min(histoMat[i][j][histoIndex],regionBlock.getHisogram()[index].normalHisto[histoIndex]);
	}
	if(sum>TB)
	{
		return true;
	}
	else
		return false;
}
void set9histo(uchar value,float*histo,int&counter)
{
	switch(value)
	{
		case 0:
			histo[0]++;
			counter++;
			break;
		case 1:
			histo[1]++;
			counter++;
			break;
		case 3:
			histo[2]++;
			counter++;
			break;
		case 7:
			histo[3]++;
			counter++;
			break;
		case 15:
			histo[4]++;
			counter++;
			break;
		case 31:
			histo[5]++;
			counter++;
			break;
		case 63:
			histo[6]++;
			counter++;
			break;
		case 127:
			histo[7]++;
			counter++;
			break;
		case 255:
			histo[8]++;
			counter++;
			break;
		}
}
void set58histo(uchar value,float*histo,int&counter)
{
	switch(value)
	{
	case 0:histo[0]++;counter++;
		break;
	case 1:histo[1]++;counter++;
		break;
	case 2:histo[2]++;counter++;
		break;
	case 3:histo[3]++;counter++;
		break;
	case 4:histo[4]++;counter++;
		break;
	case 6:histo[5]++;counter++;
		break;
	case 7:histo[6]++;counter++;
		break;
	case 8:histo[7]++;counter++;
		break;
	case 12:histo[8]++;counter++;
		break;
	case 14:histo[9]++;counter++;
		break;
	case 15:histo[10]++;counter++;
		break;
	case 16:histo[11]++;counter++;
		break;
	case 24:histo[12]++;counter++;
		break;
	case 28:histo[13]++;counter++;
		break;
	case 30:histo[14]++;counter++;
		break;
	case 31:histo[15]++;counter++;
		break;
	case 32:histo[16]++;counter++;
		break;
	case 48:histo[17]++;counter++;
		break;
	case 56:histo[18]++;counter++;
		break;
	case 60:histo[19]++;counter++;
		break;
	case 62:histo[20]++;counter++;
		break;
	case 63:histo[21]++;counter++;
		break;
	case 64:histo[22]++;counter++;
		break;
	case 96:histo[23]++;counter++;
		break;
	case 112:histo[24]++;counter++;
		break;
	case 120:histo[25]++;counter++;
		break;
	case 124:histo[26]++;counter++;
		break;
	case 126:histo[27]++;counter++;
		break;
	case 127:histo[28]++;counter++;
		break;
	case 128:histo[29]++;counter++;
		break;
	case 129:histo[30]++;counter++;
		break;
	case 131:histo[31]++;counter++;
		break;
	case 135:histo[32]++;counter++;
		break;
	case 143:histo[33]++;counter++;
		break;
	case 159:histo[34]++;counter++;
		break;
	case 191:histo[35]++;counter++;
		break;
	case 192:histo[36]++;counter++;
		break;
	case 193:histo[37]++;counter++;
		break;
	case 195:histo[38]++;counter++;
		break;
	case 199:histo[39]++;counter++;
		break;
	case 207:histo[40]++;counter++;
		break;
	case 223:histo[41]++;counter++;
		break;
	case 224:histo[42]++;counter++;
		break;
	case 225:histo[43]++;counter++;
		break;
	case 227:histo[44]++;counter++;
		break;
	case 231:histo[45]++;counter++;
		break;
	case 239:histo[46]++;counter++;
		break;
	case 240:histo[47]++;counter++;
		break;
	case 241:histo[48]++;counter++;
		break;
	case 243:histo[49]++;counter++;
		break;
	case 247:histo[50]++;counter++;
		break;
	case 248:histo[51]++;counter++;
		break;
	case 249:histo[52]++;counter++;
		break;
	case 251:histo[53]++;counter++;
		break;
	case 252:histo[54]++;counter++;
		break;
	case 253:histo[55]++;counter++;
		break;
	case 254:histo[56]++;counter++;
		break;
	case 255:histo[57]++;counter++;
		break;
	}
}
void BackGroundModel::release()
{
	for(int i=0;i<BblockVnum;++i)
	{
		//BregionBlocks[i]=(RegionBlock*)malloc(sizeof(RegionBlock)*BblockHnum);
		
		for(int j=BblockHnum-1;j>0;--j)
		{
			BregionBlocks[i][j].~RegionBlock();
		}
		free(BregionBlocks[i]);
		free(histoMat[i]);
		free(Bblock[i]);
	}
	free(BregionBlocks);
	free(histoMat);
	free(Bblock);
}
BackGroundModel::~BackGroundModel(void)
{

}
