#include "include/icp_interface.h"


__icp::__icp() {
    data_to.clear();
    data_ref.clear();
    data_shifted.clear();

    dx = 0.0f, dy = 0.0f, theta = 0.0f;

    R = Matrix::eye(3);
    T = Matrix(3, 1);
}

int __icp::set_Pt_to(vector<CvPoint3D32f> in) {

    data_to.clear();
    for (size_t i = 0; i < in.size(); i++) {
        data_to.push_back(in[i].x);
        data_to.push_back(in[i].y);
        data_to.push_back(in[i].z);
    }
    return data_to.size();
}

int __icp::set_Pt_ref(vector<CvPoint3D32f> in) {
    data_ref.clear();
    for (size_t i = 0; i < in.size(); i++) {
        data_ref.push_back(in[i].x);
        data_ref.push_back(in[i].y);
        data_ref.push_back(in[i].z);
    }
    return data_ref.size();
}

int __icp::run() {

    // 没数据就放弃计算
    if (!data_to.size() || !data_ref.size())
        return __FAILED;

    // ICP主程序
    IcpPointToPoint icp(&data_ref[0], data_ref.size() / 3, 3);
    icp.setMaxIterations(250);
    icp.setMinDeltaParam(1e-5);
    double residual = icp.fit(&data_to[0], data_to.size() / 3, R, T, -1);

    // results
    cout << endl << "Transformation results:" << endl;
    cout << "R:" << endl << R << endl << endl;
    cout << "t:" << endl << T << endl << endl;
    cout << "Residual:" << residual;

    dx = T.val[0][0];
    dy = T.val[1][0];


    return __SUCCEEDED;
}


void __icp::show_DataResult() {


}

