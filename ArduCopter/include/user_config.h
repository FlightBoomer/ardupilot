#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#pragma GCC diagnostic ignored "-Wunused-variable"

#include "../Copter.h"
#include <rplidar_sdk/rplidar.h>

#define PI		3.1415926535898979323f
#define SUCCESS 0


// 激光雷达定义
#ifndef _countof
	#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef MIN
	#define MIN(x,y) (x)<(y)?(x):(y)
#endif

#ifndef MAX
	#define MAX(x,y) (x)>(y)?(x):(y)
#endif

#define SWAP(x, y) (x ^= y ^= x ^= y)

#define ANGLE_ALL			360
#define Rplidar_Max_Range	6000		// 最大测量距离，单位: mm
#define LidarImageSize		600
#define LidarImageWidth		LidarImageSize
#define LidarImageHeight	LidarImageSize
#define LidarImageScale		0.05		// 0.10 默认: 1 / 20

typedef struct {
	_u8   Quality;
	float Angle;
	float Dst;
} __scandot;

#endif // USER_CONFIG_H
