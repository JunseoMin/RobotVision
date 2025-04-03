#include "rectification.h"

void Rectification::setParam(Eigen::Matrix3d& leftParam, Eigen::Matrix3d rightParam, 
                            std::vector<double>& lDist, std::vector<double>& rDist,
                            std::vector<Eigen::Matrix4f> lExtrinsic,
                            std::vector<Eigen::Matrix4f> rExtrinsic,
                            Eigen::MatrixXf& modelPoints
                           ){
  this -> leftParam_  = leftParam;
  this -> rightParam_ = rightParam;
  this -> lDist_      = lDist;
  this -> rDist_      = rDist;

  this -> lExtrinsic_ = lExtrinsic;
  this -> rExtrinsic_ = rExtrinsic;

  for(int i = 0; i < lExtrinsic.size(); i ++){
    Eigen::Matrix4d extrinsic;
    extrinsic = rExtrinsic.at(i).cast<double>() * lExtrinsic.at(i).inverse().cast<double>(); //  Trg * Tlg^-1 == Tlr
    Trls_.emplace_back(extrinsic);
  }
}

void Rectification::run(){
  std::cout << "rectification run!\n";
  Eigen::Matrix<double, 3, 4> P_I0;
  P_I0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 ;
  
  for (int imgIdx = 0; imgIdx < Trls_.size(); imgIdx++)
  { // 1 --> r, 2 --> l
    Eigen::Matrix4d Tr = rExtrinsic_.at(imgIdx).cast<double>();
    Eigen::Matrix4d Tl = lExtrinsic_.at(imgIdx).cast<double>();

    Eigen::Matrix<double, 3, 4> Pr = rightParam_ * P_I0 * Tr;
    Eigen::Matrix3d             Qr = Pr.block<3,3>(0,0);
    Eigen::Vector3d             qr = Pr.block<3,1>(0,3).transpose();

    // calc optical center
    Eigen::Vector3d cr = (-Qr.inverse() * qr); // reference to global frame

    Eigen::Matrix<double, 3, 4> Pl = leftParam_ * P_I0 * Tl;
    Eigen::Matrix3d             Ql = Pl.block<3,3>(0,0);
    Eigen::Vector3d             ql = Pl.block<3,1>(0,3).transpose();

    // calc optical center
    Eigen::Vector3d cl = (-Ql.inverse() * ql); // reference to global frame

    Eigen::Vector3d X = (cr - cl).normalized();                   // baseline
    Eigen::Vector3d Y = Tr.block<1,3>(2,0).cross(X).normalized(); // Tr's 3rd row means right camera's z axis reference to global frame
    Eigen::Vector3d Z = X.cross(Y).normalized();

    Eigen::Matrix3d R;  // rotation converts global -> new frame
    R << X,
         Y,
         Z;
    
    Eigen::Vector3d tr = -R * cr;
    Eigen::Vector3d tl = -R * cl;

    Eigen::Matrix3d K = rightParam_;

    Eigen::Matrix4d newTr;
    newTr << R(0,0),  R(0,1), R(0,2), tr(0),
             R(1,0),  R(1,1), R(1,2), tr(1),
             R(2,0),  R(2,1), R(2,2), tr(2),
             0     ,  0     , 0     , 1;
     
    Eigen::Matrix4d newTl;
    newTl << R(0,0),  R(0,1), R(0,2), tl(0),
             R(1,0),  R(1,1), R(1,2), tl(1),
             R(2,0),  R(2,1), R(2,2), tl(2),
             0     ,  0     , 0     , 1;
     
    Eigen::Matrix<double, 3, 4> newRproj = K * P_I0 * newTr;
    Eigen::Matrix<double, 3, 4> newLproj = K * P_I0 * newTl;

    Eigen::Matrix3d newTrSE2 = newRproj.block<3,3>(0,0) * Pr.block<3,3>(0,0).inverse();  // SE(2) matrix
    Eigen::Matrix3d newTlSE2 = newLproj.block<3,3>(0,0) * Pl.block<3,3>(0,0).inverse();  // SE(2) matrix

    newTrs_.emplace_back(newTrSE2);
    newTrs_.emplace_back(newTlSE2);

    std::cout << "new Tr:\n" << newTr << '\n';
    std::cout << "new Tl:\n" << newTl;
    std::cout << "new Tr SE(2):\n" << newTrSE2 << '\n';
    std::cout << "new Tl SE(2):\n" << newTlSE2;
  }
}