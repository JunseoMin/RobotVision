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

inline void q2Q(Eigen::Quaterniond& q, Eigen::Matrix4d& Q){
  Q.setIdentity();
  Q(0,0) = q.w(); Q(0,1) = -q.x(); Q(0,2) = -q.y(); Q(0,3) = -q.z();
  Q(1,0) = q.x(); Q(1,1) =  q.w(); Q(1,2) = -q.z(); Q(1,3) =  q.y();
  Q(2,0) = q.y(); Q(2,1) =  q.z(); Q(2,2) =  q.w(); Q(2,3) = -q.x();
  Q(3,0) = q.z(); Q(3,1) = -q.y(); Q(3,2) =  q.x(); Q(3,3) = q.w();  
}

inline void q2W(Eigen::Quaterniond& q, Eigen::Matrix4d& W){
  W.setIdentity();
  W(0,0) = q.w(); W(0,1) = -q.x(); W(0,2) = -q.y(); W(0,3) = -q.z();
  W(1,0) = q.x(); W(1,1) =  q.w(); W(1,2) =  q.z(); W(1,3) = -q.y();
  W(2,0) = q.y(); W(2,1) = -q.z(); W(2,2) =  q.w(); W(2,3) =  q.x();
  W(3,0) = q.z(); W(3,1) =  q.y(); W(3,2) = -q.x(); W(3,3) =  q.w();
}

#endif // UTILLS_HPP
