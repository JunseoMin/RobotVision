// Camera Calibration  by Zhang's Method

//---------------------------------------------------------------------------
#ifndef calibrationZhangH
#define calibrationZhangH

#define		_GAMMA_CALIB 		1
#define		_DISTORTION_CALIB	2
#define		_OPTIMISE_CALIB	    4

#define		_MAX_IMG_CALIB      50

#include "kfc.h"
#include "optima.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <ceres/ceres.h>

struct KMONOCAM_PARAM
{
  double  fu;              //  |  fu  gamma    uo |
  double  fv;              //  |  0     fv     vo |
  double  gamma;           //  |  0     0      1  |
  double	uo;
  double	vo;
  double	k1,k2,k3,k4;     // k1,k2: radial distortion;  k3,k4: tangential distortion
  int		  width,height;    //image size
};

struct Observation{
  Observation(
  double u, double v, double x, double y, double z, 
  double r11, double r12, double r13, double t1,
  double r21, double r22, double r23, double t2,
  double r31, double r32, double r33, double t3
  ) :u(u), v(v), x(x), y(y), z(z),
  r11(r11),r12(r12),r13(r13),
  r21(r21),r22(r22),r23(r23),
  r31(r31),r32(r32),r33(r33),
  t1(t1),t2(t2),t3(t3)
  {}

  double u,v,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33, t1,t2,t3;
};

struct ReprojectionError{
  ReprojectionError(const Observation& obs):obs(obs){}

  template<typename T>
  bool operator()(const T* const camera_params, T* residuals)const{  // alpha , beta , u0 , v0 , k1 , k2
    
    T alpha = camera_params[0]; // fu
    T beta  = camera_params[1]; // fv
    T u0    = camera_params[2]; // u0
    T v0    = camera_params[3]; // v0
    T k1    = camera_params[4]; // k1
    T k2    = camera_params[5]; // k2

    T cx = obs.r11 * T(obs.x) + obs.r12 * T(obs.y) + obs.r13 * T(obs.z) + obs.t1;
    T cy = obs.r21 * T(obs.x) + obs.r22 * T(obs.y) + obs.r23 * T(obs.z) + obs.t2;
    T cz = obs.r31 * T(obs.x) + obs.r32 * T(obs.y) + obs.r33 * T(obs.z) + obs.t3;

    T rx = cx / cz;
    T ry = cy / cz;
    T r2 = rx * rx + ry * ry;;

    T distortion = T(1.0) + k1 * r2 + k2 * r2 * r2;
    
    T dx = rx * distortion;
    T dy = ry * distortion;
    
    T u_proj = alpha * dx + u0;
    T v_proj = beta * dy + v0;
    
    residuals[0] = u_proj - T(obs.u);
    residuals[1] = v_proj - T(obs.v);

    return true;
  }

  Observation obs;
};

struct PointPair{
  Eigen::Vector3f pCamera;
  Eigen::Vector3f pWorld;
};

class KCalibrationZhang
{
public:
  KCalibrationZhang()   = default;

  void                  doCalib(const std::vector<Eigen::MatrixXf>& lFr, const Eigen::MatrixXf& mM);
  std::vector<float>    getEval();
  void                  getParam(Eigen::Matrix3d& intrinsic, std::vector<double>& dist);
  void                  getExtrinsic(std::vector<Eigen::Matrix4f>& extrinsic);
  void                  evalParamDiff();
  void                  evalCoordDiff();


private:
  std::vector<Eigen::Vector3f> normalizePoint_(std::vector<Eigen::Vector3f> raw, bool isCam = true); // Normalize raw points
  Eigen::Matrix3f              calcHomography_(std::vector<Eigen::Vector3f>& camNormal);
  void                         estimateIntrinsic_();
  void                         optimizeParams_();
  void                         estimateExtrinsic_();

  Eigen::Matrix3f              A_;
  Eigen::Matrix3f              TNorm_;
  Eigen::Matrix3f              TNormInv_;
  std::vector<Eigen::Matrix3f> homographies_;
  std::vector<Eigen::Vector3f> worldNormal_;  // Normalize coords
  std::vector<Eigen::Matrix4f> extrinsics_;   // SE(3) extrinsic parameter

  KMONOCAM_PARAM camParam_;                   // data for the monocamera calibration
  KMONOCAM_PARAM camParamIdeal_;              // data for the monocamera calibration

  std::vector<std::vector<Eigen::Vector3f>> camPoints_;
  std::vector<Eigen::Vector3f>              worldPoints_; // Raw points from real-world coord (mM) (x,y)

  float meanParam_;
  float stdParam_;

  float meanProj_;
  float stdProj_;
};

//---------------------------------------------------------------------------
#endif
