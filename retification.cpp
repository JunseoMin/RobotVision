#include "ratification.h"

void Ratification::setParam(Eigen::Matrix3d& leftParam, Eigen::Matrix3d rightParam, 
                            std::vector<double>& lDist, std::vector<double>& rDist,
                            std::vector<Eigen::Matrix4f> lExtrinsic,
                            std::vector<Eigen::Matrix4f> rExtrinsic,
                            Eigen::MatrixXf& modelPoints
                           ){
  this -> _leftParam  = leftParam;
  this -> _rightParam = rightParam;
  this -> _lDist      = lDist;
  this -> _rDist      = rDist;

  this -> _lExtrinsic = lExtrinsic;
  this -> _rExtrinsic = rExtrinsic;
}

void Ratification::calcRatification(){
  
}