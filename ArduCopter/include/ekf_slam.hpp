#ifndef EKF_SLAM_HPP
#define EKF_SLAM_HPP

#pragma GCC diagnostic ignored "-Wshadow"
#include "Eigen/Core"
#include "Eigen/Dense"

#include "dt.hpp"

using Eigen::MatrixXd;

// 算法详见论文

class __ekf_slam {
public:
    __ekf_slam() {

    }// __ekf_slam()

    ~__ekf_slam() {

    }// ~__ekf_slam()

    void set_SensorInput(double x_in, double y_in) { xz = x_in, yz = y_in; }

    void set_CtrlInput(double acc_x, double acc_y) { ax = acc_x, ay = acc_y; }

    void get_Output(double &x_out, double &y_out, double &vx_out, double &vy_out) { x_out = x, y_out = y, vx_out = vx, vy_out = vy; }

    void run(double x_in, double y_in, double acc_x, double acc_y) {

        set_SensorInput(x_in, y_in);
        set_CtrlInput(acc_x, acc_y);

        dt = t.get();
        run_EKF();

    }// void run()

    void reset() {
        int i;

        ///
        /// 噪声矩阵
        Q.resize(4, 4);
        Q << 0.49, 0,    0,    0,
             0,    0.49, 0,    0,
             0,    0,    0.49, 0,
             0,    0,    0,    0.49;

        R.resize(2, 2);
        R << 0.01, 0,
             0,    0.01;

        ///
        /// 协方差矩阵
        P.resize(4, 4);
        P << 1, 1, 1, 1,
             1, 1, 1, 1,
             1, 1, 1, 1,
             1, 1, 1, 1;

        ///
        /// 状态矩阵
        //X.resize(4, 1);
        X.setZero(4, 1);

        //Z.resize(4, 1);
        Z.setZero(4, 1);

        ///
        /// Jacob矩阵
        //F.resize(4, 4);
        F.setZero(4, 4);

        //W.resize(4, 4);
        W.setZero(4, 4);
        for (i = 0; i < W.cols(); i++)
            W(i, i) = 1;

        //V.resize(2, 2);
        V.setZero(2, 2);
        for (i = 0; i < V.cols(); i++)
            V(i, i) = 1;

        //J.resize(2, 4);
        J.setZero(2, 4);

        ///
        /// Kalman增益
        //K.resize(4, 2);
        K.setZero(4, 2);

    }// void reset()

private:

    double x, y;
    double x_last, y_last;
    double xz, yz;              // 传感器输入
    double vx, vy;
    double ax, ay;
    double dt;
    __dt t;

    MatrixXd X;
    MatrixXd P;
    MatrixXd K;
    MatrixXd Z;
    MatrixXd F, W, J, V;
    MatrixXd Q, R;

    void run_EKF() {

        ///
        /// 预测阶段
        X(0, 0) = X(0, 0) + X(2, 0) * dt + 0.5f * ax * dt * dt;
        X(1, 0) = X(1, 0) + X(3, 0) * dt + 0.5f * ay * dt * dt;
        X(2, 0) = X(2, 0) + ax * dt;
        X(3, 0) = X(3, 0) + ay * dt;

        F.resize(4, 4);
        F << 1,  0,  dt, 0,
             0,  1,  0,  dt,
             0,  0,  1,  0,
             0,  0,  0,  1;

        MatrixXd Ft, Wt;
        Ft = F.transpose();
        Wt = W.transpose();
        P = F * P * Ft + W * Q * Wt;

        ///
        /// 输出数据
        x  = X(0, 0);
        y  = X(1, 0);
        vx = X(2, 0);
        vy = X(3, 0);

    }// void run_EKF()


};

#endif // EKF_SLAM_HPP
