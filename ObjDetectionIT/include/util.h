#ifndef UTIL_H
#define UTIL_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<Eigen/Dense>
#include<Eigen/StdVector>

#include<math.h>
#include<time.h>

using namespace std;

namespace OBJECT
{
    double getAngle(double angle1, double angle2);
    void sort_vec(const Eigen::Vector3d &vec, Eigen::Vector3d &sorted_vec, Eigen::Vector3i &ind);
    void dual_quadric_to_ellipsoid_parameters(Eigen::Vector3d &center, Eigen::Vector3d &axes, Eigen::Matrix3d &R, Eigen::Matrix4d &Q);
}

#endif