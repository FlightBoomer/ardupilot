#ifndef ICP_INTERFACE_HPP
#define ICP_INTERFACE_HPP

#include "user_config.h"
#include <opencv2/opencv.hpp>

#include <icp_opencv/icp.h>

using namespace cv;

class __icp
{
public:

	__icp() {
		R[0] = 1.0f, R[1] = 0.0f, R[2] = 0.0f, R[3] = 1.0f;
		T[0] = T[1] = 0.0f;

		dx_temp[0] = dx_temp[1] = dx_temp[2] = 0.0f;
		dy_temp[0] = dy_temp[1] = dy_temp[2] = 0.0f;
		dyaw_temp[0] = dyaw_temp[1] = dyaw_temp[2] = 0.0f;

		d_yaw = 0;
	}// __icp()

	double dx, dy;
	double d_yaw;

	int get_Pts(vector<__scandot> in) {
		size_t i;
		CvPoint2D32f temp;
		float theta, rho;

		data_last.clear();
		for (i = 0; i < data.size(); i++) {
			temp = data[i];
			data_last.push_back(temp);
		}
		data.clear();
		for (i = 0; i < in.size(); i++) {
			__scandot dot;
			dot = in[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

			theta = dot.Angle * PI / 180;
			rho = dot.Dst;

			temp.x = rho  * sin(theta);			// * LidarImageScale + halfWidth
			temp.y = -rho * cos(theta);			// 一个主要的失真原因是作图的时候的比例尺缩放
												// 解决这个事情以后，即不缩放，就算采样一次处理一次信噪比也够
			data.push_back(temp);
		}

		return in.size();
	}// 	int get_Pts(vector<__scandot> in)

	int run(vector<__scandot> in, bool is_show) {
		int stat;

		get_Pts(in);
		stat = run(is_show);

		return stat;
	}// int run(vector<__scandot> in, bool is_show)

	int run(bool is_show) {
		const int halfWidth = LidarImageWidth / 2;
		const int halfHeight = LidarImageHeight / 2;

		//Mat image_base(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));
		Mat image(LidarImageHeight, LidarImageWidth, CV_8UC3, Scalar(0, 0, 0));
		//image = image_base.clone();

		// 没数据就放弃计算
		if (!data_last.size() || !data.size())
			return -1;
		CvMat r = cvMat(2, 2, CV_32F, R);
		CvMat t = cvMat(2, 1, CV_32F, T);

		// ICP主程序
		float err = icp(&data_last[0], data_last.size(),
						&data[0], data.size(),
						&r, &t, cvTermCriteria(CV_TERMCRIT_ITER, 45, 0.001));	// cvTermCriteria(CV_TERMCRIT_ITER, 30, 0.1)
		// 测算和保存平移数据，这段和data_shifted都可以不要其实
		data_shifted.clear();
		for (int i = 0; i < (int)data_last.size(); i++) {
			CvPoint2D32f temp;
			float x = data_last[i].x;
			float y = data_last[i].y;
			float X = (R[0] * x + R[1] * y + T[0]);
			float Y = (R[2] * x + R[3] * y + T[1]);
			temp.x = X;
			temp.y = Y;
			data_shifted.push_back(temp);
		}

		// 作图
		// 画图，顺带储存数据
		// 红的：变换后的点
		// 绿的：变换前的点
		// 蓝的：参考点
		// 理论上，红蓝应该几乎重合，绿色和蓝色不重合
		if (is_show) {
			for (size_t i = 0; i < data_last.size(); i++) {
				float x_pt = data_shifted[i].x * LidarImageScale + halfWidth;
				float y_pt = data_shifted[i].y * LidarImageScale + halfHeight;
				circle(image, Point(x_pt, y_pt), 2, Scalar(0, 0, 255), -1, 8, 0);		// R
			}
			for (size_t i = 0; i < data.size(); i++) {
				float x_pt = data[i].x * LidarImageScale + halfWidth;
				float y_pt = data[i].y * LidarImageScale + halfHeight;
				circle(image, Point(x_pt, y_pt), 2, Scalar(255, 0, 0), -1, 8, 0);		// B
			}
			for (size_t i = 0; i < data_last.size(); i++) {
				float x_pt = data_last[i].x * LidarImageScale + halfWidth;
				float y_pt = data_last[i].y * LidarImageScale + halfHeight;
				circle(image, Point(x_pt, y_pt), 2, Scalar(0, 255, 0), -1, 8, 0);		// G
			}
			imshow("icp", image);
		}

		// 关键，计算结果
		calc_dR_dYaw();

		// 打印数据，显示结果
		//PX4_INFO("err = %f", err);
		//PX4_INFO("dx: %6.3f dy: %6.3f", dx, dy);
		//PX4_INFO("d_yaw: %6.3f", d_yaw);
		//PX4_INFO("R[0]: %6,3f, R[1]: %6.3f", R[0], R[1]);

		return SUCCESS;
	}// int run(bool is_show)

private:

	vector<CvPoint2D32f> data, data_last, data_shifted;

	float R[4], T[2];				// 旋转矩阵、平移矩阵
	//CvMat r, t;					// 这个改成临时变量

	float dx_temp[3], dy_temp[3];	// 滑动平均值滤波
	float dyaw_temp[3];

	int calc_dR_dYaw() {
		//  计算平移和旋转矩阵

		// 平移矩阵
		// 滑动平均值滤波
		dx_temp[2] = dx_temp[1], dx_temp[1] = dx_temp[0], dx_temp[0] = T[0];
		dy_temp[2] = dy_temp[1], dy_temp[1] = dy_temp[0], dy_temp[0] = T[1];
		dx = (dx_temp[0] + dx_temp[1] + dx_temp[2]) / 3.0f;
		dy = (dy_temp[0] + dy_temp[1] + dy_temp[2]) / 3.0f;

		// 旋转矩阵
		float dyaw_cos = R[0];
		float dyaw_sin = R[1];
		// 因为dyaw很小，所以无需考虑-180~+180的情况，但是yaw就要了
		// 滑动平均值滤波
		dyaw_temp[2] = dyaw_temp[1], dyaw_temp[1] = dyaw_temp[0];
		//dyaw_temp[0] = (acos(dyaw_cos) + asin(dyaw_sin)) / 2.0f;		// acos有点小毛病，做不出负值，所以就很奇怪
		dyaw_temp[0] = asin(dyaw_sin);									// sin的正负注意一下
		d_yaw = (dyaw_temp[0] + dyaw_temp[1] + dyaw_temp[2]) / 3.0f;

		return SUCCESS;
	}// int calc_dR_dYaw()
};

#endif // ICP_INTERFACE_HPP
