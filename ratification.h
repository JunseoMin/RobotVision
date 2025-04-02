/**
 * @brief ratification class for robot vision 
 * @author @JunseoMin
 */

#ifndef ratificationH
#define ratificationH

#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

class Ratification
{
public:
  Ratification() = default;
  
  void calcRatification();
  void setParam(Eigen::Matrix3d& leftParam, Eigen::Matrix3d rightParam, 
                std::vector<double>& lDist, std::vector<double>& rDist,
                std::vector<Eigen::Matrix4f> lExtrinsic,
                std::vector<Eigen::Matrix4f> rExtrinsic,
                Eigen::MatrixXf& modelPoints
              );
  
  void getImage();


private:
  void _drawLine();

  // Camera params
  Eigen::Matrix3d     _leftParam;
  Eigen::Matrix3d     _rightParam;
  std::vector<double> _lDist;
  std::vector<double> _rDist;
  
  // extrinsic parameters \in SE(3)
  std::vector<Eigen::Matrix4f> _lExtrinsic;
  std::vector<Eigen::Matrix4f> _rExtrinsic;

  Eigen::Matrix4d _Trl;         // Transformation right cam -> left cam
};


#endif
