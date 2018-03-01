#ifndef __SLAM_HPP
#define __SLAM_HPP

#include "user_config.h"
#include "icp_interface.hpp"
#include "dt.hpp"

#define Map_ImgSize			(LidarImageSize * 3)
#define Map_ImgWidth		Map_ImgSize
#define Map_ImgHeight		Map_ImgSize
#define Map_ImgScale		LidarImageScale


class __slam
{
public:

    __slam() {
        t = 0;
        x = y = 0;
        x_last = y_last = 0;
        map_data.clear();
        yaw = 0;

        Mat zero(Map_ImgHeight, Map_ImgWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
        gridmap = zero.clone();
        zero.release();

    }// __slam()

    ~__slam() {
        gridmap.release();
    }// ~__slam()

    int run(vector<__scandot> in) {
        size_t i;

        current_data.clear();
        for (i = 0; i < in.size(); i++) {
            __scandot dot;
            dot = in[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

            float theta = dot.Angle * PI / 180;
            float rho = dot.Dst;

            CvPoint2D32f temp;
            temp.x = rho  * sin(theta);			// * LidarImageScale + halfWidth
            temp.y = -rho * cos(theta);			// 一个主要的失真原因是作图的时候的比例尺缩放
                                                // 解决这个事情以后，即不缩放，就算采样一次处理一次信噪比也够
            current_data.push_back(temp);
        }

        //  第一次直接记录数据即可
        if (t == 0) {
            for (i = 0; i < in.size(); i++) {

                CvPoint2D32f temp = current_data[i];
                map_data.push_back(temp);
            }
            _icp.set_Pts(in);
        }

        // 第二次开始进行匹配
        // 首先选择当前位置的某一邻域进行计算
        vector<CvPoint2D32f> map_subdata;
        map_subdata.clear();
        for (i = 0; i < map_data.size(); i++) {
            CvPoint2D32f temp = map_data[i];
            double dx = x - temp.x, dy = y - temp.y;
            double dst = sqrt(dx * dx + dy * dy);
            const double dst_threshold = 4500.0f;
            if (dst < dst_threshold)
                map_subdata.push_back(temp);
        }


        _icp.set_Pts(map_subdata);						// p.s. : 这两个东西顺序不能倒，因为set_Pts会清除旧的数据
        _icp.set_Pts_Last(current_data);		// map_data和in也不能倒，不然会发散，因为是in->map_data，这个步骤有方向性
        _icp.run(true);


        x_last = x, y_last = y;
        x = _icp.dx; y = _icp.dy;
        vx = x - x_last; vy = y - y_last;
        yaw = _icp.d_yaw;

        // 如果误差大于一定值，则认为发生了运动
        // 将新的点进行平移和旋转存储进'
        if (_icp.err >= 4500) {
            //vector<CvPoint2D32f> data_shifted = _icp.get_Data_Shifted();
            for (i = 0; i < current_data.size(); i++) {			// data_shifted.size()
                CvPoint2D32f t = current_data[i];				// 变换之间的数据
                CvPoint2D32f temp;										// 变换之后的数据
                temp.x = _icp.R[0] * t.x + _icp.R[1] * t.y + x;         // 平移-旋转变换
                temp.y = _icp.R[2] * t.x + _icp.R[3] * t.y + y;

                //首先检查栅格地图
                Point pix((int)(temp.x * Map_ImgScale + Map_ImgWidth  / 2),
                                (int)(temp.y * Map_ImgScale + Map_ImgHeight / 2));
                if (pix.x < 0 || pix.x >= gridmap.cols ||
                    pix.y < 0 || pix.y >= gridmap.rows)
                    continue;
                if (gridmap.at<Vec3b>(pix)[0] != 0)
                    continue;

                gridmap.at<Vec3b>(pix)[0] = gridmap.at<Vec3b>(pix)[1]
                                                              = gridmap.at<Vec3b>(pix)[2] = 255;
                map_data.push_back(temp);

            }
        }

        imshow("map", gridmap);

        //cout << "dx: " << x << " " << "dy: " << y << endl;
        //cout << "err: "     << _icp.err << endl;
        //cout << "dyaw: " <<_icp.R[1] << endl;

        t++;
        return (int)t;

    }// int run()

private:

    double x, y;
    double x_last, y_last;
    double vx, vy;
    double yaw;
    double t;			// 总时间

    Mat gridmap;

    __icp _icp;
    vector<CvPoint2D32f> current_data;
    vector<CvPoint2D32f> map_data;				// vector<__scandot> map_data, 原来用极坐标，结果发现极坐标有问题，很容易让数据爆炸，所以干脆改成笛卡尔坐标

    __dt tx;
};


#endif  // __SLAM_HPP

