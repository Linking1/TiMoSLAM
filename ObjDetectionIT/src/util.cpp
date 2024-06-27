#include "util.h"

namespace OBJECT
{

    double getAngle(double angle1, double angle2)
    {
        double angleSum = 0;
        if(angle1*angle2<0)
        {
            angleSum = abs(angle1) + abs(angle2);
        }
        else
        {
            angleSum = abs(angle1 - angle2);
        }

        if(angleSum > M_PI/2)
        {
            angleSum = M_PI - angleSum;
        }
        return angleSum;
    }

    void dual_quadric_to_ellipsoid_parameters(Eigen::Vector3d &center, Eigen::Vector3d &axes, Eigen::Matrix3d &R, Eigen::Matrix4d &Q)
    {
        Q = Q / (-Q(3,3));
        center(0) = -Q(0, 3);
        center(1) = -Q(1, 3);
        center(2) = -Q(2, 3);
        Eigen::Matrix4d T;
        T.setIdentity();
        T(0, 3) = -center(0);
        T(1, 3) = -center(1);
        T(2, 3) = -center(2);
        Eigen::Matrix4d Qcent = T*Q*T.transpose();

        Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(Qcent.block(0,0,3,3));
        // Eigen::Vector3cd emD = eigen_solver.eigenvalues();
        // Eigen::MatrixXcd emV = eigen_solver.eigenvectors();
        // Eigen::VectorXcd emDSort;
        // Eigen::Matrix3cd emVSort;

        Eigen::Matrix3d emD = eigen_solver.pseudoEigenvalueMatrix();
        Eigen::Matrix3d emV = eigen_solver.pseudoEigenvectors();
        Eigen::Vector3d emDSort;
        Eigen::Matrix3d emVSort;

        Eigen::Vector3d evD;
        evD << emD(0, 0), emD(1, 1), emD(2, 2);
        // 根据特征值大小进行排序
        // 排序结果在原序列中的位置
        Eigen::Vector3i ind;
        sort_vec(evD, emDSort, ind);
        emVSort << emV.block(0, ind(0), 3, 1),
                    emV.block(0, ind(1), 3, 1),
                    emV.block(0, ind(2), 3, 1);
        
        axes << sqrt(abs(emDSort(0))),
                        sqrt(abs(emDSort(1))),
                        sqrt(abs(emDSort(2)));
        
        R = emVSort;
    }

    void sort_vec(const Eigen::Vector3d &vec, Eigen::Vector3d &sorted_vec, Eigen::Vector3i &ind)
    {
        // Inputs：
        // vec：待排序的向量
        // ====================
        // Outputs：
        // sorted_vec：排序结果
        // ind：排序结果中各个元素在原始向量的位置

        ind = Eigen::Vector3i::LinSpaced(vec.size(), 0, vec.size() - 1); //[0 1 2 3 ... N-1]
        auto rule = [vec](int i, int j) -> bool {
            return vec(i) > vec(j);
        }; //正则表达式，作为sort的谓词
        std::sort(ind.data(), ind.data() + ind.size(), rule);
        //data成员函数返回VectorXd的第一个元素的指针，类似于begin()
        sorted_vec.resize(vec.size());
        for (int i = 0; i < vec.size(); i++)
        {
            sorted_vec(i) = vec(ind(i));
        }
    }

    /*
    template <class T>
    void quat_to_euler_zyx(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw)
    {
        T qw = q.w();
        T qx = q.x();
        T qy = q.y();
        T qz = q.z();

        roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        pitch = asin(2 * (qw * qy - qz * qx));
        yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    }

    template void quat_to_euler_zyx<double>(const Eigen::Quaterniond &, double &, double &, double &);
    template void quat_to_euler_zyx<float>(const Eigen::Quaternionf &, float &, float &, float &);


    template <class T>
    Eigen::Matrix<T, 3, 3> euler_zyx_to_rot(const T &roll, const T &pitch, const T &yaw)
    {
        T cp = cos(pitch);
        T sp = sin(pitch);
        T sr = sin(roll);
        T cr = cos(roll);
        T sy = sin(yaw);
        T cy = cos(yaw);

        Eigen::Matrix<T, 3, 3> R;
        R << cp * cy, (sr * sp * cy) - (cr * sy), (cr * sp * cy) + (sr * sy),
            cp * sy, (sr * sp * sy) + (cr * cy), (cr * sp * sy) - (sr * cy),
            -sp, sr * cp, cr * cp;
        return R;
    }

    template Eigen::Matrix3d euler_zyx_to_rot<double>(const double &, const double &, const double &);
    template Eigen::Matrix3f euler_zyx_to_rot<float>(const float &, const float &, const float &);

    template <class T>
    void rot_to_euler_zyx(const Eigen::Matrix<T, 3, 3> &R, T &roll, T &pitch, T &yaw)
    {
        pitch = asin(-R(2, 0));

        if (abs(pitch - M_PI / 2) < 1.0e-3)
        {
            roll = 0.0;
            yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) + roll;
        }
        else if (abs(pitch + M_PI / 2) < 1.0e-3)
        {
            roll = 0.0;
            yaw = atan2(R(1, 2) - R(0, 1), R(0, 2) + R(1, 1)) - roll;
        }
        else
        {
            roll = atan2(R(2, 1), R(2, 2));
            yaw = atan2(R(1, 0), R(0, 0));
        }
    }

    template void rot_to_euler_zyx<double>(const Eigen::Matrix3d &, double &, double &, double &);
    template void rot_to_euler_zyx<float>(const Eigen::Matrix3f &, float &, float &, float &);

    template <class T>
    void linespace(T starting, T ending, T step, std::vector<T> &res)
    {
        res.reserve((ending - starting) / step + 2);
        while (starting <= ending)
        {
            res.push_back(starting);
            starting += step; // TODO could recode to better handle rounding errors
            if (res.size() > 1000)
            {
                std::cout << "Linespace too large size!!!!" << std::endl;
                break;
            }
        }
    }
    template void linespace(int, int, int, std::vector<int> &);
    template void linespace(double, double, double, std::vector<double> &);
    */

}