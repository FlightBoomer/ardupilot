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
        is_use_ins = true;
        reset();
    }// __ekf_slam()

    ~__ekf_slam() {

    }// ~__ekf_slam()

    void set_SensorInput(double x_in, double y_in) { xz = x_in, yz = y_in; }

    void set_CtrlInput(double acc_x, double acc_y) { ax = acc_x, ay = acc_y; }

    void set_VelInsInput(double vx_in, double vy_in, bool is_use_v_in) { vx_ins = vx_in, vy_ins = vy_in; is_use_ins = is_use_v_in; }

    void set_VelLdrInput(double vx_in, double vy_in) { vx_ldr = vx_in; vy_ldr = vy_in; }

    void get_Output(double &x_out, double &y_out, double &vx_out, double &vy_out) { x_out = x, y_out = y, vx_out = vx, vy_out = vy; }

    void run() {

        //set_SensorInput(x_in, y_in);
        //set_CtrlInput(acc_x, acc_y);

        dt = t.get();
        run_EKF();

    }// void run()

    void reset() {
        int i;

        ///
        /// 计算次数
        k = 0;

        ///
        /// 噪声矩阵
        Q.resize(4, 4);
        Q << 0.49, 0, 0, 0,
            0, 0.49, 0, 0,
            0, 0, 0.49, 0,
            0, 0, 0, 0.49;

        R.resize(4, 4);
        R << 0.01, 0, 0, 0,
            0, 0.01, 0, 0,
            0, 0, 0.01, 0,
            0, 0, 0, 0.01;
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
        Z_last.setZero(4, 1);

        ///
        /// Jacob矩阵
        //F.resize(4, 4);
        F.setZero(4, 4);

        //W.resize(4, 4);
        W.setZero(4, 4);
        for (i = 0; i < W.cols(); i++)
            W(i, i) = 1;

        //V.resize(2, 2);
        V.setZero(4, 4);
        for (i = 0; i < V.cols(); i++)
            V(i, i) = 1;

        //J.resize(2, 4);
        J.setZero(4, 4);

        ///
        /// Kalman增益
        //K.resize(4, 2);
        K.setZero(4, 2);

        ///
        /// 自适应系数
        D.setZero(4, 4);
        D << 0.01, 0, 0, 0,
            0, 0.01, 0, 0,
            0, 0, 0.01, 0,
            0, 0, 0, 0.01;

    }// void reset()

private:

    double x, y;
    double x_last, y_last;
    double xz, yz;              // 传感器输入
    double vx, vy;
    double vx_ins, vy_ins;      // 通过惯性导航得到的速度
    double vx_ldr, vy_ldr;
    double ax, ay;
    double dt;
    __dt t;

    bool is_use_ins;
    int  k;                      // 计算次数

    MatrixXd X;
    MatrixXd P;
    MatrixXd K;
    MatrixXd Z, Z_last;
    MatrixXd F, W, J, V;
    MatrixXd Q, R;
    MatrixXd D;

    void run_EKF() {
        int i;

        dt /= 1000.0f;

        ///
        /// 预测阶段
        X(0, 0) = X(0, 0) + X(2, 0) * dt + 0.5f * ax * dt * dt;
        X(1, 0) = X(1, 0) + X(3, 0) * dt + 0.5f * ay * dt * dt;
        if (is_use_ins) {
            X(2, 0) = vx_ins;
            X(3, 0) = vy_ins;
        }
        X(2, 0) = X(2, 0) + ax * dt;
        X(3, 0) = X(3, 0) + ay * dt;

        F.resize(4, 4);
        F << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

        MatrixXd Ft, Wt;
        Ft = F.transpose();
        Wt = W.transpose();
        P = F * P * Ft + W * Q * Wt;

        ///
        /// 更新阶段
        Z_last = Z;
        Z(0, 0) = xz;
        Z(1, 0) = yz;
        Z(2, 0) = vx_ldr;
        Z(3, 0) = vy_ldr;

        J.setZero(4, 4);
        J << 0.6, 0, 0.4*dt, 0,
            0, 0.6, 0, 0.4*dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
        MatrixXd Jt, Vt, innov;
        Jt = J.transpose();
        Vt = V.transpose();
        innov = J * P * Jt + V * R * Vt;
        innov = innov.inverse();
        K = P * Jt * innov;

        MatrixXd I;
        I.setZero(4, 4);
        for (i = 0; i < I.cols(); i++)
            I(i, i) = 1;
        X = (I - K) * X + K * Z;

        P = (I - K * J) * P;


        if (k >= 3) {
            MatrixXd inv;
            inv = P + R;
            inv = inv.inverse();
            Q = P * inv;

            MatrixXd dZ, dZ_t;
            dZ = dZ_t = Z - Z_last;
            dZ_t = dZ.transpose();
            D = (double)(k - 2) / (double)(k - 1) *  D + 1. / (double)(k - 1) * dZ * dZ_t;
            //R = D / 2 * dZ * dZ_t - Q / 2;
            R = D;

            //P = R * P * inv;
        }


        ///
        /// 输出数据
        x = X(0, 0);
        y = X(1, 0);
        vx = X(2, 0);
        vy = X(3, 0);

        k++;

        cout << P << endl << endl;
        cout << "ekf: x: " << x << " y: " << y << endl;

    }// void run_EKF()


};

#endif // EKF_SLAM_HPP
