#include "StdAfx.h"
#include <opencv2/opencv.hpp>
#include <iostream>

;using namespace cv;
using namespace std;
class DistanceDescriptor
{
private:
	float Position;
	double VertDistance;
	double Distance;
	float k;
public:
	int GetPosition();
	double GetVertDistance();
	double GetDistance();
	float Getk();
	void SetPosition(int position);
	void SetVertDistance(double _VertDistance);
	void SetDistance(double _Distance);
	void Setk(float _k);
	void ComputeDistance(double scale);
	DistanceDescriptor(float steelwirePosition,double _VertDistance,float _k);
	DistanceDescriptor(void);
	~DistanceDescriptor(void);
};

