#ifndef ICP_INTERFACE_H
#define ICP_INTERFACE_H

#include "config.h"
#include "misc_tools.hpp"

#include <vector>
#include <opencv2/opencv.hpp>

#include "../libraries/icp/icpPointToPlane.h"       // #include <icp/icpPointToPlane.h>
#include "../libraries/icp/icpPointToPoint.h"
#include "../libraries/icp/matrix.h"



using namespace std;
using namespace cv;

class __icp {
public:
    __icp();

    int set_Pt_to(vector<CvPoint3D32f> in);
    int set_Pt_ref(vector<CvPoint3D32f> in);

    void orientate_Pt_to(double a);					// 匹配前预旋转参考数据
    void orientate_Pt_ref(double a);					// 注意，这两个函数必须输入弧度值

    void get_Pos_Shifted(double &x, double &y) { x = dx, y = dy; }
    void get_Theta_Shifted(double &angle) { angle = theta; }

    int run();

    void show_DataResult();

private:

    vector<double> data_to;							// 平移前数据，icp算法第一个参数
    vector<double> data_ref;							// 平移后数据，icp算法第二个参数
    vector<double> data_shifted;						// 平移后数据进行平移得到的数据

    double dx, dy;
    double theta;										// 角度

    Matrix R;
    Matrix T;

};


#endif // ICP_INTERFACE_H
