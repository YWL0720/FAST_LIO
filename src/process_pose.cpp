//
// Created by ywl on 23-11-30.
//
#include <Eigen/Core>
#include <fstream>
#include <Eigen/Geometry>
#include <iostream>


int main(int argc, char** argv)
{
    std::string filename = std::string(std::string(ROOT_DIR) + "map/pose.json");
    std::string new_filename = std::string(std::string(ROOT_DIR) + "map/pose_new.json");

    std::fstream new_file;
    new_file.open(new_filename,  std::ios::out);
    new_file.precision(9);
    new_file.setf(std::ios::fixed);

    std::fstream file;
    file.open(filename);

    Eigen::Matrix4d init_T = Eigen::Matrix4d::Identity();
    double x, y, z, qw, qx, qy, qz;
    file >> x >> y >> z >> qw >> qx >> qy >> qz;
    Eigen::Quaterniond init_q = {qw, qx, qy, qz};
    init_T.block<3, 3>(0, 0) = init_q.toRotationMatrix();
    init_T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    new_file << 0 << " " << 0 << " " << 0 << " " << 1 << " " << 0 << " " << 0 << " " << 0 << std::endl;

    while (file >> x >> y >> z >> qw >> qx >> qy >> qz)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q = {qw, qx, qy, qz};
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

        Eigen::Matrix4d new_T = Eigen::Matrix4d::Identity();
        new_T = init_T.inverse() * T;
        Eigen::Quaterniond new_q(new_T.block<3, 3>(0, 0));
        new_file << new_T.block<3, 1>(0, 3)[0] << " " << new_T.block<3, 1>(0, 3)[1] << " " << new_T.block<3, 1>(0, 3)[2] << " "
        << new_q.w() << " " << new_q.x() << " " << new_q.y() << " " << new_q.z() << std::endl;
    }
    std::cout << "Done" << std::endl;

}