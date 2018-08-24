#include "StdAfx.h"
#include "DistanceDescriptor.h"

void DistanceDescriptor::ComputeDistance(double scale)
{
	Distance=abs((VertDistance/k)*sqrt((k*k-1)))*scale;
	//Distance=sin(k)*VertDistance*scale;
}
double DistanceDescriptor::GetVertDistance()
{
	return VertDistance;
}
int DistanceDescriptor::GetPosition()
{
	return Position;
}
double DistanceDescriptor::GetDistance()
{
	return Distance;
}
float DistanceDescriptor::Getk()
{
	return k;
}
void DistanceDescriptor::SetVertDistance(double _VertDistance)
{
	VertDistance=_VertDistance;
}
void DistanceDescriptor::SetPosition(int position)
{
	Position=position;
}
void DistanceDescriptor::SetDistance(double _Distance)
{
	Distance=_Distance;
}
void DistanceDescriptor::Setk(float _k)
{
	k=_k;
}
DistanceDescriptor::DistanceDescriptor(float steelwirePosition,double _VertDistance,float _k)
{
	Position=steelwirePosition;
	VertDistance=_VertDistance;
	k=_k;
}
DistanceDescriptor::DistanceDescriptor(void)
{
}
DistanceDescriptor::~DistanceDescriptor(void)
{
}