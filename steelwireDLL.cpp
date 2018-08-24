// steelwireDLL.cpp : 定义 DLL 应用程序的导出函数。
/*
安迪
2018-7-15

钢丝绳检测的总体思路：
利用LBP动态监测，建立LBP检测模型（初始化背景模型过程），将运动的钢丝绳前景提
取出来（backGroundModel.frontDetect(grayFrame,outPutImage);），之后将提取的
前景二值化（in=outPutImage>BIN_THRESHOLD;）成为黑白图像(动态物体为白值)，为
了防止误检，当二值图的直方图白值和黑值的比例落在[START_LOW,START_HIGH]之间时
就被认定为检测到运动钢丝绳（if(startindex<=START_HIGH&&startindex>=START_LOW)），
对于这部分的视频帧，利用getSteelwire(outPutImage,leftP,rightP)函数得到钢丝绳整个前景的最左点leftP和
最右点rightP;利用两点并使用getRegion得到钢丝绳的整体范围，利用得到的范围在原始帧中取钢丝绳的
轮廓二值（阈值为BIN_POINTCHECK_THRESHOLD，一般在100左右），随后调用pointCheck函
数，它对轮廓二值图闭运算后检出左右边缘左右边缘使用直线拟合后得到该处的钢丝斜
率，利用斜率对左右边缘坐标差进行修正，结果存放在DistanceDescriptorList中并输出到txt文件中

当前存在一个缺陷。受到检测原理局限性的影响，如果钢丝绳并没有做上或下运动，而是单纯左右晃动，程序也会把当前
帧识别为运动帧并且为它们计算位置坐标，这样最后的位置可能有误差。所以在点检的时候尽量避免钢丝绳单纯的左右晃动
和上下时相对相机的前后晃动。这两种晃动都会影响最后的结果。
*/

#include "stdAfx.h"
#include "steelwireDLL.h"
#include "DistanceDescriptor.h"

#include <iostream>
#include <fstream> 
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#define BIN_THRESHOLD 0
#define BIN_POINTCHECK_THRESHOLD 105
#define REGION_RADIUS_TIMES 1.2
#define REGION_RADIUS_CHECKPOINT 50
#define START_LOW 0.02
#define START_HIGH 0.2
#define PI 3.1415926535
using namespace std;
using namespace cv; 



/**********************函数声明开始***************************/
void ImageDevide(Mat &inputimage,int fragmentNum,vector<Mat>&Mats);
void getEdge(Mat& mat);
void getDistanceDescriptor(
	vector<Mat>& Mats,
	float steelwirePosition,
	vector<cv::Point>&leftEdgePoints,
	vector<Point>&rightEdgePoints,
	vector<DistanceDescriptor>& DistanceDescriptorList,
	double scale);
void getSteelwire(Mat&image,Point &leftP,Point& rightP);
void getRegion(Mat& grayFrame,Point& leftP,Point& rightP,Point &regionLeftP,Point &regionRightP);
void pointCheck(Mat& input,Rect&region,vector<DistanceDescriptor>&outputVector,int counter,float framesPerS,float v,float videoTimes,float scale);
void drawLine(cv::Mat &image, double theta, double rho, cv::Scalar color);
vector<std::string> split(string str,string pattern);
/**********************函数声明结束****************************/




ofstream outfile;//用于输出测量数据txt，输出文件名称为视频输入文件名称。
ofstream logfile("log.txt");//用于输出日志文件，日志文件存放程序处理过程和每一帧的识别状态。
float DistanceSum=0;//记录前3帧的像素宽度和（前三帧钢丝绳像素宽度取平均，得到标尺像素）
int frameindex=0;//帧索引，记录已经处理的帧数。

/*
参数1：_scale钢丝绳直径数据，点检前测出
参数2：videoname文件路径
参数3：VideoTimes视频的倍率X1或X2
参数4：v钢丝绳提升速率（m/s)或（cm/s）函数中使用的是m/s
参数5：fps视频帧率 25.0或30.0
*/
STEELWIREDLL_API int measureWidth(float _scale,const char* videopath,float VideoTimes,float v,float fps)
{
	//videoname

	/*******解析视频文件名**********/
	string pattern="\\";
	vector<std::string> result=split(videopath,pattern);
	string name=result.at(result.size()-1);
	result.clear();
	result=split(name,".");
	if(result.size()==2)
	name=result.at(0);
	else{
		logfile<<"文件名无效:"<<videopath<<endl;
		return 0;
	}
	name.append(".txt");//name="xxx.txt"
	outfile.open(name);
	/*******视频文件名解析完成******/

	/*******初始化数据******/
	int framecounter=0;
	//钢丝绳实测宽度,输入的参数，需要在每次进行点检时，由工人实际测量，
	//这个实测的宽度会与前三帧检测的像素宽度换算为本次点检使用的尺度。
	float scale=_scale;
	string imagelink=videopath;
	Mat frame;
	Mat grayFrame;
	Mat firstFrame;
	Mat initFrame;
	Mat outPutImage;
	Mat SteelwireRIO;
	Mat in;
	vector<DistanceDescriptor> DistanceDescriptorList;//钢丝绳宽度描述符列表，
	//计算出的每个位置的钢丝绳宽度将存在DistanceDescriptorList中

	Point leftP;
	Point rightP;
	Point regionLeftP;
	Point regionRightP;
	uchar _58table[256];
	logfile<<"执行函数..."<<"\n"<<endl;
	VideoCapture capture;
	capture.open(videopath);
	logfile<<"VideoCapture 初始化完成"<<"\n"<<endl;
	/*******初始化数据完成******/

	/************************************************/
	


	if(capture.isOpened())
	{
		logfile<<"得到视频文件:"<<videopath<<endl;
	}
	else
	{
		logfile<<"未得到视频文件:"<<videopath<<endl;
		return 0;
	}

	capture>>firstFrame;
	/********************初始化背景模型************************/
	logfile<<"初始化背景模型..."<<endl;
	lbp58table(_58table);
	BackGroundModel backGroundModel(firstFrame,12,12);//12为LBP直方图块的尺寸大小，越大抗噪性越好，一般取10或12，偶数
	backGroundModel.setInitframe(firstFrame);
	backGroundModel.setLBPmod(_58table);
	backGroundModel.calculateLBP();
	backGroundModel.calculateInitHisto();
	/********************初始化背景模型完成************************/

	while(1)
	{
		capture.read(frame);//等价于cap.read(frame) 
		
		if(frame.empty())//如果某帧为空则退出循环
             break;
		frameindex++;
		logfile<<"处理第"<<frameindex<<"帧；";
		imshow("video", frame);
		cvtColor( frame, grayFrame, CV_BGR2GRAY );
		backGroundModel.frontDetect(grayFrame,outPutImage);
		imshow("前景", outPutImage);

		backGroundModel.updateBackGroundModel(grayFrame);
		/*****************************************/

		const int channels[1] = { 0 };
		//直方图的每一个维度的 柱条的数目（就是将灰度级分组）  
		int histSize[] = { 256 };   //如果这里写成int histSize = 256;   那么下面调用计算直方图的函数的时候，该变量需要写 &histSize  
		//定义一个变量用来存储 单个维度 的数值的取值范围    
		float midRanges[] = { 0, 256 };
 
		//确定每个维度的取值范围，就是横坐标的总数    
		const float *ranges[] = { midRanges };
 
		//输出的结果存储的 空间 ，用MatND类型来存储结果  
		MatND dstHist;
		in=outPutImage>BIN_THRESHOLD;
		calcHist(&in, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);
 
		//calcHist  函数调用结束后，dstHist变量中将储存了 直方图的信息  用dstHist的模版函数 at<Type>(i)得到第i个柱条的值  at<Type>(i, j)得到第i个并且第j个柱条的值    
 
		//首先先创建一个黑底的图像，为了可以显示彩色，所以该绘制图像是一个8位的3通道图像    
		Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
		Mat graydrawImage;
 
		//一个图像的某个灰度级的像素个数（最多为图像像素总数），可能会超过显示直方图的所定义的图像的尺寸，因此绘制直方图的时候，让直方图最高的地方只有图像高度的90%来显示  
 
		//先用minMaxLoc函数来得到计算直方图后的像素的最大个数    
		double g_dHistMaxValue;
		minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
 
		//遍历直方图得到的数据    
		for (int i = 0; i < 256; i++)
		{
			int value = cvRound(256 * 0.9 *(dstHist.at<float>(i) / g_dHistMaxValue));
 
			line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 0, 0));
		}
		float startindex=dstHist.at<float>(255)/dstHist.at<float>(0);
		//outfile<<startindex<<" "<<endl;

	/*****************************************/
		if(startindex<=START_HIGH&&startindex>=START_LOW)
		{

			framecounter++;
			logfile<<"第"<<frameindex<<"帧识别为动态；动态第"<<framecounter<<"帧"<<endl;
			getSteelwire(outPutImage,leftP,rightP);
			getRegion(grayFrame,leftP,rightP,regionLeftP,regionRightP);
			SteelwireRIO=grayFrame(Rect(regionLeftP.x,0,regionRightP.x-regionLeftP.x,grayFrame.rows));
			//imshow("video compare", SteelwireRIO);
			int checkpostion=SteelwireRIO.rows/2;
			Rect region(0,checkpostion-REGION_RADIUS_CHECKPOINT,SteelwireRIO.cols,REGION_RADIUS_CHECKPOINT*2);
			pointCheck(SteelwireRIO,region,DistanceDescriptorList,framecounter,fps, v,VideoTimes,scale);			
		}
		else
		{
			logfile<<"第"<<frameindex<<"帧未被识别为动态"<<endl;
		}
		waitKey(10);//每帧延时10毫秒
		if (cvWaitKey(20) == 27)  
		{
			logfile<<"ESC退出"<<endl;
			break;
		}
	}
	logfile<<"释放背景模型"<<endl;
	backGroundModel.release();
	logfile<<"背景模型释放完成"<<endl;
	logfile<<"释放视频资源"<<endl;
	capture.release();
	logfile<<"视频资源释放完成"<<endl;
	logfile<<"窗口释放"<<endl;
	cvDestroyAllWindows();
	logfile<<"窗口释放完成"<<endl;
    return 1;  
}
void ImageDevide(Mat &inputimage,int fragmentNum,vector<Mat>&Mats)
{
	int fragmentSize=inputimage.rows/fragmentNum;
	if(fragmentNum<1)
	{
		return;
	}
	if(fragmentNum>inputimage.rows)
	{
		fragmentNum=inputimage.rows;
	}
	int counter=0;
	while(counter!=fragmentNum)
	{
		Mat imageROI;
		imageROI=inputimage(Rect(0,fragmentSize*counter,inputimage.cols,MIN(inputimage.rows-fragmentSize*counter,fragmentSize)));
		Mats.push_back(imageROI);
		counter++;
	}
}
void getEdge(Mat& mat,vector<Point>&leftEdgePoints,vector<Point>&rightEdgePoints)
{
	for(int y=0;y<mat.rows;y++)
	{
		int lowMark=0;
		while(lowMark<mat.cols-1&&!mat.at<uchar>(y,lowMark))
			lowMark++;
		leftEdgePoints.push_back(Point(lowMark-1,y));
		int highMark=mat.cols-1;

		while(!mat.at<uchar>(y,highMark)&&highMark>lowMark)
			highMark--;
		rightEdgePoints.push_back(Point(highMark+1,y));
		if(lowMark<highMark)
		{
			for(int i=lowMark+1;i<highMark;i++)
			{
				mat.at<uchar>(y,i)=0;
			}
		}
		else
		{
			for(int i=0;i<mat.cols-1;i++)
			{
				mat.at<uchar>(y,i)=0;
			}
		}
	}
}
/*
运算钢丝绳的点检位置的宽度描述符
参数1：将点检位置横向划分的多个Mat存放在Mats中
参数2：点检位置在钢丝绳的坐标
参数3：钢丝绳左侧轮廓线点集
参数4：钢丝绳右侧轮廓线点集

*/
void getDistanceDescriptor(vector<Mat>& Mats,float steelwirePosition,vector<cv::Point>&leftEdgePoints,vector<Point>&rightEdgePoints,vector<DistanceDescriptor>& DistanceDescriptorList,double scale)
{
	for(int i=0;i<Mats.size();i++)
	{
		Mat pop;
		pop=Mats.at(i);
		//getEdge(pop);

		Vec4f left_line_para,right_line_para; 
		fitLine(Mat(leftEdgePoints), left_line_para,  CV_DIST_L2 , 0, 1e-2, 1e-2);
		fitLine(Mat(rightEdgePoints),right_line_para, CV_DIST_L2, 0, 1e-2, 1e-2);

		double left_cos_theta = left_line_para[0];
		double left_sin_theta = left_line_para[1];
		double left_x0 = left_line_para[2], left_y0 = left_line_para[3];
		double left_phi = atan2(left_sin_theta, left_cos_theta) + PI / 2.0;
		double left_rho = left_y0 * left_cos_theta - left_x0 * left_sin_theta;
		drawLine(Mats.at(i), left_phi, left_rho, cv::Scalar(0));
		double left_k = left_sin_theta / left_cos_theta;
		double left_b = left_y0 - left_k * left_x0;
		double left_x = 0;
		double left_y = left_k * left_x + left_b;

		double right_cos_theta = right_line_para[0];
		double right_sin_theta = right_line_para[1];
		double right_x0 = right_line_para[2], right_y0 = right_line_para[3];
		double right_phi = atan2(right_sin_theta, right_cos_theta) + PI / 2.0;
		double right_rho = right_y0 * right_cos_theta - right_x0 * right_sin_theta;
		drawLine(Mats.at(i), right_phi, right_rho, cv::Scalar(0));
		double right_k = right_sin_theta / right_cos_theta;
		double right_b = right_y0 - right_k * right_x0;
		double right_x = 0;
		double right_y = right_k * right_x + right_b;

		imshow("", Mats.at(i));

		float distanceSum=0.0;//用于计算平均值
		float avrtheta=(right_k+left_k)/2;
		for(int y=0;y<Mats.at(i).rows;y++)
		{
			int lowMark=0;
			while(lowMark<Mats.at(i).cols-1&&!Mats.at(i).at<uchar>(y,lowMark))//左点集
				lowMark++;
			int highMark=Mats.at(i).cols-1;
			while(!Mats.at(i).at<uchar>(y,highMark)&&highMark>lowMark)//优点集
				highMark--;
			DistanceDescriptor  Descriptor(steelwirePosition,highMark-lowMark+1,avrtheta);
			Descriptor.ComputeDistance(scale);
			DistanceDescriptorList.push_back(Descriptor);
			distanceSum+=Descriptor.GetDistance();
		}
		cout<<"位置："<<steelwirePosition<<" 宽度："<<distanceSum/Mats.at(i).rows<<endl;
		outfile<<frameindex<<"\t"<<steelwirePosition<<"\t"<<distanceSum/Mats.at(i).rows<<endl;
	} 
	cout<<""<<endl;
}

void getSteelwire(Mat&image,Point &leftP,Point &rightP)
{

	int lowTh=60;
	Mat bw = image > BIN_THRESHOLD;

	


	Canny(bw,image,lowTh,3*lowTh,3);
    vector<vector<Point>> contours;  
    vector<Vec4i> hierarchy;
    findContours(bw,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());

    //imageContours=Mat::zeros(imageContours.size(),CV_8UC1); //存放绘制轮廓线 
    Mat Contours=Mat::zeros(image.size(),CV_8UC1);  //存放绘制轮廓点

	int cmin=1000;
	int cmax=10000;
	vector<vector<Point>>::const_iterator itc=contours.begin();
	vector<vector<Point>>::const_iterator itmax=contours.begin();
	int max=0;
	int postion=0;
	int maxPotion=0;
	if(contours.size()!=0)
	{
		while(itc!=contours.end())
		{
			if(itc->size()>max)
			{
				max=itc->size();
				itmax=itc;
				maxPotion=postion;
			}
			++postion;
			++itc;
		}
		int maxX=0;
		int minX=image.cols;
		for(int j=0;j<itmax->size();j++)   
		{  
			 //绘制出contours向量内所有的像素点
			if(contours[maxPotion][j].x>maxX)
			{
				maxX=contours[maxPotion][j].x;
				rightP.x=maxX;
				rightP.y=contours[maxPotion][j].y;
			}
			else if(contours[maxPotion][j].x<minX)
			{
				minX=contours[maxPotion][j].x;
				leftP.x=minX;
				leftP.y=contours[maxPotion][j].y;
			}
			 Point P=Point(contours[maxPotion][j].x,contours[maxPotion][j].y);
			 Contours.at<uchar>(P)=255;
		}
	}
	else
	{
		leftP.x=0;
		rightP.x=image.cols;
	}
}
void getRegion(Mat& grayFrame,Point& inputleftP,Point& inputrightP,Point &outregionLeftP,Point &outregionRightP)
{
		float radius;
		radius=(inputrightP.x-(inputrightP.x+inputleftP.x)/2)*REGION_RADIUS_TIMES;
		outregionLeftP.x=MAX((inputrightP.x+inputleftP.x)/2-radius,0);
		outregionLeftP.y=0;
		outregionRightP.x=MIN((inputrightP.x+inputleftP.x)/2+radius,grayFrame.cols);
		outregionRightP.y=grayFrame.rows;
}
/*
根据点检时钢丝速度、视频帧率、视频倍速、点检前初始的标尺大小等参数计算出点检位置，宽度
参数1：输入的点检位置图像矩阵input

*/
void pointCheck(Mat& input,Rect&region,vector<DistanceDescriptor>&outputVector,int counter,float framesPerS,float v,float videoTimes,float scale)
{
	int lowTh=30;
	float postion=videoTimes*(counter/framesPerS)*v;

	Mat checkMat;
	Mat Contours;
	Mat bw = input < BIN_POINTCHECK_THRESHOLD;
	

	/***********************闭运算开始****************************
	闭运算消除掉bw做阈值运算后出现的小孔洞*/
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(bw, Contours, MORPH_CLOSE, element);
	imshow("闭运算后",Contours);
	/***********************闭运算结束****************************/
	checkMat=Contours(Rect(0,region.y,input.cols,region.height));
	/************************直方图开始*****************************/
	const int channels[1] = { 0 };
		//直方图的每一个维度的 柱条的数目（就是将灰度级分组）  
		int histSize[] = { 256 };   //如果这里写成int histSize = 256;   那么下面调用计算直方图的函数的时候，该变量需要写 &histSize  
		//定义一个变量用来存储 单个维度 的数值的取值范围    
		float midRanges[] = { 0, 256 };
 
		//确定每个维度的取值范围，就是横坐标的总数    
		const float *ranges[] = { midRanges };
 
		//输出的结果存储的 空间 ，用MatND类型来存储结果  
		MatND dstHist;
		Mat in;
		in=input(Rect(0,region.y,input.cols,region.height));
		calcHist(&in, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);
 
		//calcHist  函数调用结束后，dstHist变量中将储存了 直方图的信息  用dstHist的模版函数 at<Type>(i)得到第i个柱条的值  at<Type>(i, j)得到第i个并且第j个柱条的值    
 
		//首先先创建一个黑底的图像，为了可以显示彩色，所以该绘制图像是一个8位的3通道图像    
		Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
		Mat graydrawImage;
 
		//一个图像的某个灰度级的像素个数（最多为图像像素总数），可能会超过显示直方图的所定义的图像的尺寸，因此绘制直方图的时候，让直方图最高的地方只有图像高度的90%来显示  
 
		//先用minMaxLoc函数来得到计算直方图后的像素的最大个数    
		double g_dHistMaxValue;
		minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
 
		//遍历直方图得到的数据    
		for (int i = 0; i < 256; i++)
		{
			int value = cvRound(256 * 0.9 *(dstHist.at<float>(i) / g_dHistMaxValue));
 
			line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 0, 0));
		}
		imshow("直方图",drawImage);
	/*************************************************************/

	imshow("check region",checkMat);
	vector<Point> leftEdgePoints;
	vector<Point> rightEdgePoints;
	getEdge(checkMat,leftEdgePoints,rightEdgePoints);//得到点检位置左右轮廓的点集
	imshow("EDGE",checkMat);
	vector<Mat> Mats;
	ImageDevide(checkMat,1,Mats);
	if(counter<=3)
	{
		getDistanceDescriptor(Mats,postion,leftEdgePoints,rightEdgePoints,outputVector,1);
		DistanceSum+=outputVector.at(counter).GetDistance();
	}else{
		getDistanceDescriptor(Mats,postion,leftEdgePoints,rightEdgePoints,outputVector,scale/(DistanceSum/3));
	}
}
void drawLine(cv::Mat &image, double theta, double rho, cv::Scalar color)
{
    if (theta < PI/4. || theta > 3.*PI/4.)// ~vertical line
    {
        cv::Point pt1(rho/cos(theta), 0);
        cv::Point pt2((rho - image.rows * sin(theta))/cos(theta), image.rows);
        cv::line( image, pt1, pt2, cv::Scalar(255), 1);
    }
    else
    {
        cv::Point pt1(0, rho/sin(theta));
        cv::Point pt2(image.cols, (rho - image.cols * cos(theta))/sin(theta));
        cv::line(image, pt1, pt2, color, 1);
    }
}
/*字符串分割函数用在分割
字符串分割函数*/
vector<std::string> split(string str,string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str+=pattern;//扩展字符串以方便操作
    int size=str.size();

    for(int i=0; i<size; i++)
    {
        pos=str.find(pattern,i);
        if(pos<size)
        {
            std::string s=str.substr(i,pos-i);
            result.push_back(s);
            i=pos+pattern.size()-1;
        }
    }
    return result;
}