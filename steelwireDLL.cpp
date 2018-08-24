// steelwireDLL.cpp : ���� DLL Ӧ�ó���ĵ���������
/*
����
2018-7-15

��˿����������˼·��
����LBP��̬��⣬����LBP���ģ�ͣ���ʼ������ģ�͹��̣������˶��ĸ�˿��ǰ����
ȡ������backGroundModel.frontDetect(grayFrame,outPutImage);����֮����ȡ��
ǰ����ֵ����in=outPutImage>BIN_THRESHOLD;����Ϊ�ڰ�ͼ��(��̬����Ϊ��ֵ)��Ϊ
�˷�ֹ��죬����ֵͼ��ֱ��ͼ��ֵ�ͺ�ֵ�ı�������[START_LOW,START_HIGH]֮��ʱ
�ͱ��϶�Ϊ��⵽�˶���˿����if(startindex<=START_HIGH&&startindex>=START_LOW)����
�����ⲿ�ֵ���Ƶ֡������getSteelwire(outPutImage,leftP,rightP)�����õ���˿������ǰ���������leftP��
���ҵ�rightP;�������㲢ʹ��getRegion�õ���˿�������巶Χ�����õõ��ķ�Χ��ԭʼ֡��ȡ��˿����
������ֵ����ֵΪBIN_POINTCHECK_THRESHOLD��һ����100���ң���������pointCheck��
��������������ֵͼ������������ұ�Ե���ұ�Եʹ��ֱ����Ϻ�õ��ô��ĸ�˿б
�ʣ�����б�ʶ����ұ�Ե����������������������DistanceDescriptorList�в������txt�ļ���

��ǰ����һ��ȱ�ݡ��ܵ����ԭ������Ե�Ӱ�죬�����˿����û�����ϻ����˶������ǵ������һζ�������Ҳ��ѵ�ǰ
֡ʶ��Ϊ�˶�֡����Ϊ���Ǽ���λ�����꣬��������λ�ÿ������������ڵ���ʱ���������˿�����������һζ�
������ʱ��������ǰ��ζ��������ֻζ�����Ӱ�����Ľ����
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



/**********************����������ʼ***************************/
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
/**********************������������****************************/




ofstream outfile;//���������������txt������ļ�����Ϊ��Ƶ�����ļ����ơ�
ofstream logfile("log.txt");//���������־�ļ�����־�ļ���ų�������̺�ÿһ֡��ʶ��״̬��
float DistanceSum=0;//��¼ǰ3֡�����ؿ�Ⱥͣ�ǰ��֡��˿�����ؿ��ȡƽ�����õ�������أ�
int frameindex=0;//֡��������¼�Ѿ������֡����

/*
����1��_scale��˿��ֱ�����ݣ����ǰ���
����2��videoname�ļ�·��
����3��VideoTimes��Ƶ�ı���X1��X2
����4��v��˿���������ʣ�m/s)��cm/s��������ʹ�õ���m/s
����5��fps��Ƶ֡�� 25.0��30.0
*/
STEELWIREDLL_API int measureWidth(float _scale,const char* videopath,float VideoTimes,float v,float fps)
{
	//videoname

	/*******������Ƶ�ļ���**********/
	string pattern="\\";
	vector<std::string> result=split(videopath,pattern);
	string name=result.at(result.size()-1);
	result.clear();
	result=split(name,".");
	if(result.size()==2)
	name=result.at(0);
	else{
		logfile<<"�ļ�����Ч:"<<videopath<<endl;
		return 0;
	}
	name.append(".txt");//name="xxx.txt"
	outfile.open(name);
	/*******��Ƶ�ļ����������******/

	/*******��ʼ������******/
	int framecounter=0;
	//��˿��ʵ����,����Ĳ�������Ҫ��ÿ�ν��е��ʱ���ɹ���ʵ�ʲ�����
	//���ʵ��Ŀ�Ȼ���ǰ��֡�������ؿ�Ȼ���Ϊ���ε��ʹ�õĳ߶ȡ�
	float scale=_scale;
	string imagelink=videopath;
	Mat frame;
	Mat grayFrame;
	Mat firstFrame;
	Mat initFrame;
	Mat outPutImage;
	Mat SteelwireRIO;
	Mat in;
	vector<DistanceDescriptor> DistanceDescriptorList;//��˿������������б�
	//�������ÿ��λ�õĸ�˿����Ƚ�����DistanceDescriptorList��

	Point leftP;
	Point rightP;
	Point regionLeftP;
	Point regionRightP;
	uchar _58table[256];
	logfile<<"ִ�к���..."<<"\n"<<endl;
	VideoCapture capture;
	capture.open(videopath);
	logfile<<"VideoCapture ��ʼ�����"<<"\n"<<endl;
	/*******��ʼ���������******/

	/************************************************/
	


	if(capture.isOpened())
	{
		logfile<<"�õ���Ƶ�ļ�:"<<videopath<<endl;
	}
	else
	{
		logfile<<"δ�õ���Ƶ�ļ�:"<<videopath<<endl;
		return 0;
	}

	capture>>firstFrame;
	/********************��ʼ������ģ��************************/
	logfile<<"��ʼ������ģ��..."<<endl;
	lbp58table(_58table);
	BackGroundModel backGroundModel(firstFrame,12,12);//12ΪLBPֱ��ͼ��ĳߴ��С��Խ������Խ�ã�һ��ȡ10��12��ż��
	backGroundModel.setInitframe(firstFrame);
	backGroundModel.setLBPmod(_58table);
	backGroundModel.calculateLBP();
	backGroundModel.calculateInitHisto();
	/********************��ʼ������ģ�����************************/

	while(1)
	{
		capture.read(frame);//�ȼ���cap.read(frame) 
		
		if(frame.empty())//���ĳ֡Ϊ�����˳�ѭ��
             break;
		frameindex++;
		logfile<<"�����"<<frameindex<<"֡��";
		imshow("video", frame);
		cvtColor( frame, grayFrame, CV_BGR2GRAY );
		backGroundModel.frontDetect(grayFrame,outPutImage);
		imshow("ǰ��", outPutImage);

		backGroundModel.updateBackGroundModel(grayFrame);
		/*****************************************/

		const int channels[1] = { 0 };
		//ֱ��ͼ��ÿһ��ά�ȵ� ��������Ŀ�����ǽ��Ҷȼ����飩  
		int histSize[] = { 256 };   //�������д��int histSize = 256;   ��ô������ü���ֱ��ͼ�ĺ�����ʱ�򣬸ñ�����Ҫд &histSize  
		//����һ�����������洢 ����ά�� ����ֵ��ȡֵ��Χ    
		float midRanges[] = { 0, 256 };
 
		//ȷ��ÿ��ά�ȵ�ȡֵ��Χ�����Ǻ����������    
		const float *ranges[] = { midRanges };
 
		//����Ľ���洢�� �ռ� ����MatND�������洢���  
		MatND dstHist;
		in=outPutImage>BIN_THRESHOLD;
		calcHist(&in, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);
 
		//calcHist  �������ý�����dstHist�����н������� ֱ��ͼ����Ϣ  ��dstHist��ģ�溯�� at<Type>(i)�õ���i��������ֵ  at<Type>(i, j)�õ���i�����ҵ�j��������ֵ    
 
		//�����ȴ���һ���ڵ׵�ͼ��Ϊ�˿�����ʾ��ɫ�����Ըû���ͼ����һ��8λ��3ͨ��ͼ��    
		Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
		Mat graydrawImage;
 
		//һ��ͼ���ĳ���Ҷȼ������ظ��������Ϊͼ�����������������ܻᳬ����ʾֱ��ͼ���������ͼ��ĳߴ磬��˻���ֱ��ͼ��ʱ����ֱ��ͼ��ߵĵط�ֻ��ͼ��߶ȵ�90%����ʾ  
 
		//����minMaxLoc�������õ�����ֱ��ͼ������ص�������    
		double g_dHistMaxValue;
		minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
 
		//����ֱ��ͼ�õ�������    
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
			logfile<<"��"<<frameindex<<"֡ʶ��Ϊ��̬����̬��"<<framecounter<<"֡"<<endl;
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
			logfile<<"��"<<frameindex<<"֡δ��ʶ��Ϊ��̬"<<endl;
		}
		waitKey(10);//ÿ֡��ʱ10����
		if (cvWaitKey(20) == 27)  
		{
			logfile<<"ESC�˳�"<<endl;
			break;
		}
	}
	logfile<<"�ͷű���ģ��"<<endl;
	backGroundModel.release();
	logfile<<"����ģ���ͷ����"<<endl;
	logfile<<"�ͷ���Ƶ��Դ"<<endl;
	capture.release();
	logfile<<"��Ƶ��Դ�ͷ����"<<endl;
	logfile<<"�����ͷ�"<<endl;
	cvDestroyAllWindows();
	logfile<<"�����ͷ����"<<endl;
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
�����˿���ĵ��λ�õĿ��������
����1�������λ�ú��򻮷ֵĶ��Mat�����Mats��
����2�����λ���ڸ�˿��������
����3����˿����������ߵ㼯
����4����˿���Ҳ������ߵ㼯

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

		float distanceSum=0.0;//���ڼ���ƽ��ֵ
		float avrtheta=(right_k+left_k)/2;
		for(int y=0;y<Mats.at(i).rows;y++)
		{
			int lowMark=0;
			while(lowMark<Mats.at(i).cols-1&&!Mats.at(i).at<uchar>(y,lowMark))//��㼯
				lowMark++;
			int highMark=Mats.at(i).cols-1;
			while(!Mats.at(i).at<uchar>(y,highMark)&&highMark>lowMark)//�ŵ㼯
				highMark--;
			DistanceDescriptor  Descriptor(steelwirePosition,highMark-lowMark+1,avrtheta);
			Descriptor.ComputeDistance(scale);
			DistanceDescriptorList.push_back(Descriptor);
			distanceSum+=Descriptor.GetDistance();
		}
		cout<<"λ�ã�"<<steelwirePosition<<" ��ȣ�"<<distanceSum/Mats.at(i).rows<<endl;
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

    //imageContours=Mat::zeros(imageContours.size(),CV_8UC1); //��Ż��������� 
    Mat Contours=Mat::zeros(image.size(),CV_8UC1);  //��Ż���������

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
			 //���Ƴ�contours���������е����ص�
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
���ݵ��ʱ��˿�ٶȡ���Ƶ֡�ʡ���Ƶ���١����ǰ��ʼ�ı�ߴ�С�Ȳ�����������λ�ã����
����1������ĵ��λ��ͼ�����input

*/
void pointCheck(Mat& input,Rect&region,vector<DistanceDescriptor>&outputVector,int counter,float framesPerS,float v,float videoTimes,float scale)
{
	int lowTh=30;
	float postion=videoTimes*(counter/framesPerS)*v;

	Mat checkMat;
	Mat Contours;
	Mat bw = input < BIN_POINTCHECK_THRESHOLD;
	

	/***********************�����㿪ʼ****************************
	������������bw����ֵ�������ֵ�С�׶�*/
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(bw, Contours, MORPH_CLOSE, element);
	imshow("�������",Contours);
	/***********************���������****************************/
	checkMat=Contours(Rect(0,region.y,input.cols,region.height));
	/************************ֱ��ͼ��ʼ*****************************/
	const int channels[1] = { 0 };
		//ֱ��ͼ��ÿһ��ά�ȵ� ��������Ŀ�����ǽ��Ҷȼ����飩  
		int histSize[] = { 256 };   //�������д��int histSize = 256;   ��ô������ü���ֱ��ͼ�ĺ�����ʱ�򣬸ñ�����Ҫд &histSize  
		//����һ�����������洢 ����ά�� ����ֵ��ȡֵ��Χ    
		float midRanges[] = { 0, 256 };
 
		//ȷ��ÿ��ά�ȵ�ȡֵ��Χ�����Ǻ����������    
		const float *ranges[] = { midRanges };
 
		//����Ľ���洢�� �ռ� ����MatND�������洢���  
		MatND dstHist;
		Mat in;
		in=input(Rect(0,region.y,input.cols,region.height));
		calcHist(&in, 1, channels, Mat(), dstHist, 1, histSize, ranges, true, false);
 
		//calcHist  �������ý�����dstHist�����н������� ֱ��ͼ����Ϣ  ��dstHist��ģ�溯�� at<Type>(i)�õ���i��������ֵ  at<Type>(i, j)�õ���i�����ҵ�j��������ֵ    
 
		//�����ȴ���һ���ڵ׵�ͼ��Ϊ�˿�����ʾ��ɫ�����Ըû���ͼ����һ��8λ��3ͨ��ͼ��    
		Mat drawImage = Mat::zeros(Size(256, 256), CV_8UC3);
		Mat graydrawImage;
 
		//һ��ͼ���ĳ���Ҷȼ������ظ��������Ϊͼ�����������������ܻᳬ����ʾֱ��ͼ���������ͼ��ĳߴ磬��˻���ֱ��ͼ��ʱ����ֱ��ͼ��ߵĵط�ֻ��ͼ��߶ȵ�90%����ʾ  
 
		//����minMaxLoc�������õ�����ֱ��ͼ������ص�������    
		double g_dHistMaxValue;
		minMaxLoc(dstHist, 0, &g_dHistMaxValue, 0, 0);
 
		//����ֱ��ͼ�õ�������    
		for (int i = 0; i < 256; i++)
		{
			int value = cvRound(256 * 0.9 *(dstHist.at<float>(i) / g_dHistMaxValue));
 
			line(drawImage, Point(i, drawImage.rows - 1), Point(i, drawImage.rows - 1 - value), Scalar(255, 0, 0));
		}
		imshow("ֱ��ͼ",drawImage);
	/*************************************************************/

	imshow("check region",checkMat);
	vector<Point> leftEdgePoints;
	vector<Point> rightEdgePoints;
	getEdge(checkMat,leftEdgePoints,rightEdgePoints);//�õ����λ�����������ĵ㼯
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
/*�ַ����ָ�����ڷָ�
�ַ����ָ��*/
vector<std::string> split(string str,string pattern)
{
    std::string::size_type pos;
    std::vector<std::string> result;
    str+=pattern;//��չ�ַ����Է������
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