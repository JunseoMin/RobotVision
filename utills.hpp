#ifndef UTILLS_HPP
#define UTILLS_HPP

#include <Eigen/Dense>
#include <Eigen/Core>

inline void makeOrthogonal(Eigen::Matrix3d& rotation){
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  rotation = U * V.transpose();
}

inline void makeOrthogonal(Eigen::Matrix2d& rotation){
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();

  rotation = U * V.transpose();
}


inline void inverseSO3(Eigen::Matrix3d& rotation){
  rotation = rotation.transpose();
}

#endif // UTILLS_HPP
