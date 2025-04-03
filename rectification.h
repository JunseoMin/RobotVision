/**
 * @brief rectification class for robot vision 
 * @author @JunseoMin
 */

#ifndef rectificationH
#define rectificationH

#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <iostream>

class Rectification
{
public:
  Rectification() = default;
  
  void setParam(Eigen::Matrix3d& leftParam, Eigen::Matrix3d rightParam, 
                std::vector<double>& lDist, std::vector<double>& rDist,
                std::vector<Eigen::Matrix4f> lExtrinsic,  // Contains global frame's pose reference to camera (Tlg)
                std::vector<Eigen::Matrix4f> rExtrinsic,  // Contains global frame's pose reference to camera (Trg)
                Eigen::MatrixXf& modelPoints
              );
  void setImages();
  void getImages();
  void run();


private:
  void drawLine_();
  void calcRectification_();  

  // Camera params
  Eigen::Matrix3d     leftParam_;
  Eigen::Matrix3d     rightParam_;
  std::vector<double> lDist_;
  std::vector<double> rDist_;
  
  // extrinsic parameters \in SE(3)
  std::vector<Eigen::Matrix4f> lExtrinsic_;
  std::vector<Eigen::Matrix4f> rExtrinsic_;

  std::vector<Eigen::Matrix4d> Trls_;       // Transformation left cam -> right cam

  std::vector<Eigen::Matrix3d> newTrs_;
  std::vector<Eigen::Matrix3d> newTls_;
};


#endif
