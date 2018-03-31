#ifndef __SLAM_H

#define __SLAM_H

#include "user_config.h"
#include "icp_interface.hpp"
#include "ekf_slam.hpp"
#include "ranging.hpp"

#include <opencv2/opencv.hpp>

#define Map_ImgSize		(LidarImageSize * 3)
#define Map_ImgWidth		Map_ImgSize
#define Map_ImgHeight		Map_ImgSize
#define Map_ImgScale		LidarImageScale
#ifndef WIN32

#else
    #include <Windows.h>
    #include <time.h>
#endif

class __slam
{
public:

    __slam() {
        t = 0;
        x = y = 0;
        x_last = y_last = 0;
        map_data.clear();
        yaw_icp = 0;

        Mat zero(Map_ImgHeight, Map_ImgWidth, CV_8UC3, Scalar(0, 0, 0));		// 图片格式： BGR
        gridmap = zero.clone();
        zero.release();

        ekf.reset();

    }// __slam::__slam()

    ~__slam() {
        gridmap.release();
    }// __slam::~__slam()

     /// 传入飞机姿态角
    void set_VehicleAtt(double roll_in, double pitch_in, double yaw_in) {
        roll = roll_in; pitch = pitch_in; yaw = yaw_in;
    }

    /// 传入地面坐标系的加速度
    void set_VehicleAcc_ef(double ax_ef, double ay_ef) {
        acc_x = ax_ef; acc_y = ay_ef;
    }

    /// 传入机体坐标系的加速度
    void set_VehicleAcc_body(double ax_body, double ay_body) {
        acc_x_body = ax_body; acc_y_body = ay_body;
        double z_virtual;
        //rotation_mat_body2ins(acc_x_body, acc_y_body, 0, acc_x, acc_y, z_virtual, roll, pitch, yaw);
    }

    int run(vector<__scandot> in) {
        size_t i;

        vector<Point> pt_in;
        for (i = 0; i < in.size(); i++) {
            __scandot dot;
            dot = in[i];		// 未滤波:Data[i] 滤波后:data_dst[i]

            float theta = dot.Angle * PI / 180;
            float rho = dot.Dst;

            Point temp;
            temp.x = rho  * sin(theta);			// * LidarImageScale + halfWidth
            temp.y = -rho * cos(theta);			// 一个主要的失真原因是作图的时候的比例尺缩放
                                                // 解决这个事情以后，即不缩放，就算采样一次处理一次信噪比也够
            pt_in.push_back(temp);
        }

        int stat = run(pt_in);					// 主程序
        return stat;

    }// int __slam::run()

    int run(vector<Point> pt_in) {
        size_t i;

        vector<CvPoint2D32f> in;		// 分成Point和CvPoint2D32f, 其实是一个东西, 做的这么复杂主要是考虑兼容性
        current_data.clear();

        double a = -roll;
        double b = -pitch;
        double g = -yaw;
        double dv0[9];
        dv0[0] = cos(b) * cos(g);
        dv0[3] = cos(a) * sin(g) + sin(a) * sin(b) * cos(g);
        dv0[6] = sin(a) * sin(g) - cos(a) * sin(b) * cos(g);
        dv0[1] = -cos(b) * sin(g);
        dv0[4] = cos(a) * cos(g) - sin(a) * sin(b) * sin(g);
        dv0[7] = sin(a) * cos(g) + cos(a) * sin(b) * sin(g);
        dv0[2] = sin(b);
        dv0[5] = -sin(a) * cos(b);
        dv0[8] = cos(a) * cos(b);

        for (i = 0; i < pt_in.size(); i++) {
            CvPoint2D32f temp;
            //temp.x = pt_in[i].x;
            //temp.y = pt_in[i].y;

            // 锁定偏航
            //temp.x = pt_in[i].x * cos(yaw) + pt_in[i].y * -sin(yaw);
            //temp.y = pt_in[i].x * sin(yaw) + pt_in[i].y * cos(yaw);

            // 平面转换
            temp.x = pt_in[i].x * dv0[0] + pt_in[i].y * dv0[3];
            temp.y = pt_in[i].x * dv0[1] + pt_in[i].y * dv0[4];

            in.push_back(temp);
            current_data.push_back(temp);
        }

        //  第一次直接记录数据即可
        if (t == 0) {
            for (i = 0; i < in.size(); i++) {

                CvPoint2D32f temp = current_data[i];
                map_data.push_back(temp);
            }
            _icp.set_Pts(in);
            pt_recorded.push_back(Point(0, 0));		// 原点是一个记录地图的点
        }

        // 第二次开始进行匹配
        // 首先选择当前位置的某一邻域进行计算
        _icp.set_Pts(map_data);						// p.s. : 这两个东西顺序不能倒，因为set_Pts会清除旧的数据
        _icp.set_Pts_Last(current_data);            // map_data和in也不能倒，不然会发散，因为是in->map_data，这个步骤有方向性
        _icp.run(true);

        /// 获得数据
        x_last = x, y_last = y;
        x = _icp.dx; y = _icp.dy;
        vx = x - x_last; vy = y - y_last;
        yaw_icp_last = yaw_icp;
        yaw_icp = _icp.d_yaw;

        /// 检查icp是否正常
        //  是则覆盖数据
    /*
        set_ranging_data(in);
        if (is_icp_functioning_error() == true && t != 0) {
            x = x_last + r.get_dx();
            y = y_last + r.get_dy();
            vx = x - x_last; vy = y - y_last;
            yaw_icp = yaw_icp_last;
        }
    */

        /// 数据代入卡尔曼滤波器
        ekf.run(x, y, 0, 0);

        if (is_ok_for_mapping(x, y)) {
            //vector<CvPoint2D32f> data_shifted = _icp.get_Data_Shifted();
            for (i = 0; i < current_data.size(); i++) {			// data_shifted.size()
                CvPoint2D32f t = current_data[i];				// 变换之间的数据
                CvPoint2D32f temp;										// 变换之后的数据
                temp.x = _icp.R[0] * t.x + _icp.R[1] * t.y + x;         // 平移-旋转变换
                temp.y = _icp.R[2] * t.x + _icp.R[3] * t.y + y;

                map_data.push_back(temp);

            }
            pt_recorded.push_back(Point(x, y));
        }


        //imshow("map", gridmap);

        cout << "dx: " << x << " " << "dy: " << y << endl;
        //cout << "err: "     << _icp.err << endl;
        //cout << "dyaw: " <<_icp.R[1] << endl;
        //cout << "yaw_icp: " << yaw_icp * 180.0f / PI << endl;

        t++;
        return (int)t;
    }// int __slam::run(vector<Point> in)

private:

    double x, y;
    double x_last, y_last;
    vector<Point> pt_recorded;				// 有效的截图点
    double vx, vy;
    double acc_x, acc_y;				// 地面坐标系下
    double acc_x_body, acc_y_body;		// 机体坐标系下
    double roll, pitch, yaw;
    double yaw_icp, yaw_icp_last;
    double t;						// 总时间

    Mat gridmap;

    __icp _icp;
    __ranging r;
    __ekf_slam ekf;
    vector<CvPoint2D32f> current_data;
    vector<CvPoint2D32f> map_data;				// vector<__scandot> map_data, 原来用极坐标，结果发现极坐标有问题，很容易让数据爆炸，所以干脆改成笛卡尔坐标

    void set_ranging_data(vector<__scandot> in) { r.set_LidarData(in, true); }

    bool is_icp_functioning_error() {

        const double err_modifier_threshold = 2.5f;

        double dx_icp, dy_icp;
        double dx_r, dy_r;			// 直接测量得到的移动量
        bool is_x_error, is_y_error;
        bool is_x_not_function, is_y_not_function;

        dx_icp = x - x_last;
        dy_icp = y - y_last;

        r.run();
        dx_r = r.get_dx();
        dy_r = r.get_dy();

        is_x_error = is_y_error = false;
        is_x_not_function = is_y_not_function = false;
        if (dx_r != 0.0f) {
            if (fabs(dx_icp / dx_r) >= err_modifier_threshold)
                is_x_error = true;
        }
        else
            is_x_not_function = true;

        if (dy_r != 0.0f) {
            if (fabs(dy_icp / dy_r) >= err_modifier_threshold)
                is_y_error = true;
        }
        else
            is_y_not_function = true;

        if (is_x_not_function && is_y_not_function)
            return true;
        else if (is_x_not_function && !is_y_not_function) {
            if (is_y_error)
                return true;
        }
        else if (!is_x_not_function && is_y_not_function) {
            if (is_x_error)
                return true;
        }
        else if (!is_x_not_function && !is_y_not_function) {
            if (is_x_error && is_y_error)
                return true;
        }
        else
            return false;
    }// bool __slam::is_icp_functioning_error()

    bool is_ok_for_mapping(double current_x, double current_y) {
        // 计算距离
        // 根据子图的位置判断添加距离
        // 子图的记录点必须远离其他已知的记录点
        size_t i;
        const double dst_threshold = 250.0f;		// 单位: mm

        for (i = 0; i < pt_recorded.size(); i++) {
            double dst;
            double dx, dy;
            dx = current_x - pt_recorded[i].x;
            dy = current_y - pt_recorded[i].y;
            dst = sqrt(dx * dx + dy * dy);
            if (dst < dst_threshold)
                return false;
        }

        return true;
    }
};




#endif	/* __SLAM_H */
