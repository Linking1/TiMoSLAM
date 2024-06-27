/**************
Author： Wenbin
Date： 2023.03.26
Description： Use the mouse to get bounding box
	and use IT to get 3D cuboid.
***************/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<Eigen/Dense>
#include<math.h>
#include<Eigen/StdVector>

// #include "Optimizer.h"

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d/features2d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "obj2d.h"
#include "obj3d.h"
#include "line_seg.h"

using namespace std;
using namespace OBJECT;

cv::Mat img;
bool select_flag = false;
cv::Rect m_select;
cv::Point origin;
int ROI_count;
char temp[100];
char rgb[100];


// Mouse function
void onMouseRectPicking(int event, int x, int y, int, void*)
{
	if (select_flag)
	{
		m_select.x = MIN(origin.x, x);
		// 不一定要等鼠标弹起才计算矩形框，而应该在鼠标按下开始到弹起这段时间实时计算所选矩形框
		m_select.y = MIN(origin.y, y);
		m_select.width = abs(x - origin.x);
		// 算矩形宽度和高度
		m_select.height = abs(y - origin.y);
		m_select &= cv::Rect(0, 0, img.cols, img.rows);
		cv::imshow("capframe", img);
	}

	if (event == CV_EVENT_LBUTTONDOWN) // 鼠标左键按下时
	{
		select_flag = true;
		// 鼠标按下的标志赋真值
		origin = cv::Point(x, y);
		// 保存下来单击捕捉到的点
		m_select = cv::Rect(x, y, 0, 0);
		cv::imshow("capframe", img);
	}

	else if (event == CV_EVENT_LBUTTONUP) // 鼠标左键抬起时
	{
		select_flag = false;
		ROI_count++;
		cv::imshow("capframe", img);

		int center_x = origin.x + (x - origin.x) / 2;
		int center_y = origin.y + (y - origin.y) / 2;
		// 计算中心点坐标
		cout << m_select.x << " " << m_select.y << " " << m_select.width << " " << m_select.height << endl;
	}
}


int main(int argc, char **argv)
{

    
    Eigen::Matrix3d K;
	// TUM3
    K << 535.4,     0, 320.1,
             0, 539.2, 247.6,
             0,     0,     1;

    
    // img = cv::imread("../data/1.png",CV_LOAD_IMAGE_UNCHANGED);
    
    img = cv::imread(argv[1],CV_LOAD_IMAGE_UNCHANGED);
    
    bool stop = false;

	cv::namedWindow("capframe", CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback("capframe", onMouseRectPicking, 0);

    while (!stop)
	{
		img = cv::imread(argv[1]);
		cv::rectangle(img, m_select, cv::Scalar(255, 0, 0), 2, 8, 0);  // 画矩形框
		cv::imshow("capframe", img);

		char key = static_cast<char>(cv::waitKey(30));
		if (key == 27)
			stop = true;
	}
    img = cv::imread(argv[1]);
    cv::Mat grayImage;
    cvtColor(img, grayImage, CV_BGR2GRAY);

    cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
    // 检测线段
    vector<cv::Vec4f> lines_std;
    // Detect the lines
    ls->detect(grayImage, lines_std);//这里把检测到的直线线段都存入了lines_std中，4个float的值，分别为起止点的坐标

	// 生成2D目标观测对象
	double x1, y1, x2, y2;
	x1 = m_select.x;
	y1 = m_select.y;
	x2 = m_select.x + m_select.width;
	y2 = m_select.y + m_select.height;

	cout << m_select.x << " " << m_select.y << " " << m_select.width << " " << m_select.height << endl;

	m_select.x = x1;
	m_select.y = y1;
	m_select.width = x2-x1;
	m_select.height = y2-y1;
	OBJECT::Obj2D* pObj2d = new OBJECT::Obj2D(0, x1, y1, x2, y2, 0, 1.0, lines_std, img, K);
	// pObj2d->FrameRot = pFrame->pObj3D->R;
	pObj2d->getObj3D(K);

	// pFrame->DrawObj(img);
	// 绘制
	pObj2d->DrawObj(img);

	pObj2d->DrawLines(img);

	cv::rectangle(img, m_select, cv::Scalar(255, 0, 0), 2, 8, 0);  // 画矩形框
	// ls->drawSegments(img, lines_std);

	imshow("capframe", img);
	cv::waitKey(0);

	return 0;
    
}
