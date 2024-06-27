/**************
Author： Wenbin
Date： 2023.03.26
Description： Use the mouse to get bounding box
	and use IT to get 3D cuboid.
***************/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>
#include <Eigen/StdVector>

// #include "Optimizer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

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

using namespace std;


int main(int argc, char **argv)
{

	// string yaml_path = "../config/tum.yaml";
    // cv::FileStorage fsSettings(yaml_path.c_str(), cv::FileStorage::READ);
    // if(!fsSettings.isOpened())
    // {
    //    cerr << "Failed to open settings file at: " << yaml_path << endl;
    //    exit(-1);
    // }

	// mKeyFrameSize = fSettings[""];
    // mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    // mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    // mPointSize = fSettings["Viewer.PointSize"];
    // mCameraSize = fSettings["Viewer.CameraSize"];
    // mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

	Eigen::Matrix3d K;
	K << 481.20,       0, 319.50,
	          0,  480.00, 239.50,
			  0,       0,      1;

	// read rgb list
	string rgblist_txt = "../../ICL/living_room_traj2_frei_png/rgb1.txt";
	ifstream rgblist_f;
	rgblist_f.open(rgblist_txt.c_str());
	// while(!f.eof())
	for (int _imid=0; _imid<881; _imid++)
	{
		string s;
		getline(rgblist_f, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			string im_path, im_name;
			ss >> im_name;
			ss >> im_path;
			cout << "---------------------------------------" << endl;
			cout << _imid << ", im path: " << im_path << endl;
			im_path = "../../ICL/living_room_traj2_frei_png/rgb/";
			// read image
			cv::Mat img = cv::imread(im_path + im_name+".png");

			// detect lines
		    // cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_ADV);
		    cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_NONE);
			// 检测线段
    		vector<cv::Vec4f> lines_std;
			cv::Mat grayImage;
    		cvtColor(img, grayImage, CV_BGR2GRAY);
			ls->detect(grayImage, lines_std);

			// read BoundingBox
			string bb_txt = "../../ICL/living_room_traj2_frei_png/box/" + im_name + ".txt";
			vector<cv::Rect> bbs;
			vector<double> confs;
			vector<int> labels;
			ifstream bb_f;
			bb_f.open(bb_txt.c_str());
			while(!bb_f.eof())
			{
				string bb_s;
				getline(bb_f, bb_s);
				if (!bb_s.empty())
				{
					cv::Rect bb_temp;
					double conf_temp;
					int label_temp;
					stringstream bb_ss;
					bb_ss << bb_s;
					bb_ss >> bb_temp.x;
					bb_ss >> bb_temp.y;
					bb_ss >> bb_temp.width;
					bb_ss >> bb_temp.height;
					bb_ss >> conf_temp;
					bb_ss >> label_temp;
					bbs.push_back(bb_temp);
				}
			}

			// detect boxes
			for(int bb_i=0; bb_i<(int)bbs.size();bb_i++)
			{
				cout << "box id: " << bb_i << endl;
				if(bbs[bb_i].width*bbs[bb_i].height<100)
				{
					continue;
				}
				// bbs[bb_i].width = bbs[bb_i].width - bbs[bb_i].x;
				// bbs[bb_i].height = bbs[bb_i].height - bbs[bb_i].y;
			    // cv::rectangle(img, bbs[bb_i], cv::Scalar(255, 0, 0), 2, 8, 0);  // 画矩形框
				double x1, y1, x2, y2;
				x1 = bbs[bb_i].x;
				y1 = bbs[bb_i].y;
				x2 = bbs[bb_i].width;
				y2 = bbs[bb_i].height;

				OBJECT::Obj2D* pObj2d = new OBJECT::Obj2D(0, x1, y1, x2, y2, 0, 1.0, lines_std, img, K);
				try
				{
					pObj2d->getObj3D(K);
				}
				catch(const std::exception& e)
				{
					continue;
				}
				
    			

				// write the cuboid into txt
				if(pObj2d->pObj3D)
				{
					string res_txt = "../../ICL/living_room_traj2_frei_png/cube/" + im_name +".txt";
					// cout << res_txt << endl;
					ofstream res_f;
					res_f.open(res_txt.c_str());
					res_f << pObj2d->pObj3D->R << endl;
					res_f << pObj2d->pObj3D->Axes.transpose() << endl;
					res_f << pObj2d->pObj3D->Center.transpose() << endl;
					res_f.close();
				}
				
				pObj2d->DrawObj(img);
    			pObj2d->DrawLines(img);
				delete pObj2d;
			}
			cv::imwrite("../../ICL/living_room_traj2_frei_png/cube/" + im_name +".png",img);
			imshow("capframe", img);
    		cv::waitKey(0);
		}
	}
	
	return 0;
}
