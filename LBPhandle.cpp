#include "stdafx.h" 
#include "LBPhandle.h"
int getHopCount(uchar i)  
{  
    int a[8]={0};  
    int k=7;  
    int cnt=0;  
    while(i)  
    {  
        a[k]=i&1;  
        i>>=1;  
        --k;
    }  
    for(int k=0;k<8;++k)  
    {  
        if(a[k]!=a[k+1==8?0:k+1])  
        {  
            ++cnt;  
        }  
    }  
    return cnt;  
}  
void lbp36table(uchar* _36table)//旋转不变的LBP
{
	memset(_36table,0,256);
	for(int i=0;i<256;++i)
	{
		int min=255;
		for(int j=0;j<8;++j)
		{
			uchar a=(i << (8 - j) | (i >> j));
			if(a<min)
			{
				min=a;
			}
		}
		_36table[min]=min; 
	} 

}
void lbp58table(uchar* table)//uniform LBP
{  
    memset(table,0,256);  
    //uchar temp=1;  
    for(int i=0;i<256;++i)  
    {  
        if(getHopCount(i)<=2)  
        {  
            table[i]=i; 
        } 
    }  
}  
void lbp9table(uchar* _58table,uchar* _36table,uchar* _9table)
{
	memset(_9table,0,256); 
	for(int i=0;i<256;++i)
	{
		if(_58table[i]==_36table[i])
		{
			_9table[i]=_58table[i];
		}
	}
}
void LBP(Mat& _src, Mat& _dst,uchar* table)
{
	_dst.create(_src.rows,_src.cols,CV_8UC1);
	_dst.setTo(0);
	for(int j=1;j<_src.cols-1;j++)
	{
        for(int i=1;i<_src.rows-1;i++) 
		{  
			uchar neighborhood[8]={0};
            uchar center = _src.at<uchar>(i,j);    
			int Sift=ALPHA;
			neighborhood[7]=_src.at<uchar>(i-1,j-1)-Sift;
			neighborhood[6]=_src.at<uchar>(i-1,j  )-Sift;
			neighborhood[5]=_src.at<uchar>(i-1,j+1)-Sift;
			neighborhood[4]=_src.at<uchar>(i  ,j+1)-Sift;
			neighborhood[3]=_src.at<uchar>(i+1,j+1)-Sift;
			neighborhood[2]=_src.at<uchar>(i+1,j  )-Sift;
			neighborhood[1]=_src.at<uchar>(i+1,j-1)-Sift;
			neighborhood[0]=_src.at<uchar>(i  ,j-1)-Sift;
			uchar temp=0; 
			for(int k=0;k<8;k++)  
            {  
                temp+=(neighborhood[k]>=center)<<k;  
            }
			_dst.at<uchar>(i,j)=table[temp];
		}
	}
}