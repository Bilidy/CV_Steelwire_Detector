#include "stdafx.h" 
#include "BackGroundModel.h"
#include "LBPhandle.h"
#ifdef STEELWIREDLL_EXPORTS  
#define STEELWIREDLL_API __declspec(dllexport) 
#else  
#define STEELWIREDLL_API __declspec(dllimport)
#endif  

STEELWIREDLL_API int measureWidth(float _scale,const char* videoname,float VideoTimes,float v,float fps)