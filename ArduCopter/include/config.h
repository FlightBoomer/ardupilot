#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#define PI 3.1415926535897932384626

#define __SUCCEEDED     0
#define __FAILED		1
#define __STANDBY		2
#define __FINISHED      4

#define LidarImageSize   600
#define LidarImageWidth  LidarImageSize
#define LidarImageHeight LidarImageSize
#define LidarImageScale  0.025				// 0.025, 默认: A1早期版本，6m半径——1 / 20

typedef struct {
    double dst;
    double angle;
} __scandot;

#endif // CONFIG_H
