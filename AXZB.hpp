#ifndef AXZB_HPP
#define AXZB_HPP

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <utills.hpp>

#include <ceres/ceres.h>

struct Observation{
  Observation(double x, double y, double u, double v, double alpha, double beta, double u0, double v0, 
              double r11, double r12, double r13, double t1,
              double r21, double r22, double r23, double t2,
              double r31, double r32, double r33, double t3
              )
              :r11(r11), r12(r12), r13(r13), t1(t1),
              r21(r21), r22(r22), r23(r23), t2(t2),
              r31(r31), r32(r32), r33(r33), t3(t3),
              x(x), y(y), u(u), v(v), alpha(alpha), beta(beta), u0(u0), v0(v0)
              {}
  
  double r11, r12, r13;
  double r21, r22, r23;
  double r31, r32, r33;
  double t1, t2, t3;
  double u, v;
  double x, y;
  double alpha, beta, u0, v0;
};

struct ImageReprojectionError{
  ImageReprojectionError(const Observation& obs):obs(obs){}

  template<typename T>
  bool operator()(const T* const params, T* residuals) const{
    //X^-1
    T ax = params[0];   // alpha
    T bx = params[1];   // beta
    T gx = params[2];   // gamma
    T txx = params[3];  // tx
    T txy = params[4];  // ty
    T txz = params[5];  // tz

    T r11x = std::cos(bx) * std::cos(gx);
    T r12x = -std::cos(bx) * std::sin(gx);
    T r13x = std::sin(bx);
    T r21x = std::sin(ax) * std::sin(bx) * std::cos(gx) + std::cos(ax) * std::sin(gx);
    T r22x = -std::sin(ax) * std::sin(bx) * std::sin(gx) + std::cos(ax) * std::cos(gx);
    T r23x = -std::sin(ax) * std::cos(bx);
    T r31x = -std::cos(ax) * std::sin(bx) * std::cos(gx) + std::sin(ax) * std::sin(gx);
    T r32x = std::cos(ax) * std::sin(bx) * std::sin(gx) + std::sin(ax) * std::cos(gx);
    T r33x = std::cos(ax) * std::cos(bx);

    // Z
    T az = params[6];   // alpha z
    T bz = params[7];   // beta z
    T gz = params[8];   // gamma z
    T txz = params[9];  // txz
    T tyz = params[10]; // tyz
    T tzz = params[11]; // txz

    T r11z = std::cos(bz) * std::cos(gz);
    T r12z = -std::cos(bz) * std::sin(gz);
    T r13z = std::sin(bz);
    T r21z = std::sin(az) * std::sin(bz) * std::cos(gz) + std::cos(az) * std::sin(gz);
    T r22z = -std::sin(az) * std::sin(bz) * std::sin(gz) + std::cos(az) * std::cos(gz);
    T r23z = -std::sin(az) * std::cos(bz);
    T r31z = -std::cos(az) * std::sin(bz) * std::cos(gz) + std::sin(az) * std::sin(gz);
    T r32z = std::cos(az) * std::sin(bz) * std::sin(gz) + std::sin(az) * std::cos(gz);
    T r33z = std::cos(az) * std::cos(bz);

    T r11_res = r11x * obs.r11 * r11z + r11x * obs.r12 * r21z + r11x * obs.r13 * r31z
              + r12x * obs.r21 * r11z + r12x * obs.r22 * r21z + r12x * obs.r23 * r31z
              + r13x * obs.r31 * r11z + r13x * obs.r32 * r21z + r13x * obs.r33 * r31z;
    T r12_res = r11x * obs.r11 * r12z + r11x * obs.r12 * r22z + r11x * obs.r13 * r32z
              + r12x * obs.r21 * r12z + r12x * obs.r22 * r22z + r12x * obs.r23 * r32z
              + r13x * obs.r31 * r12z + r13x * obs.r32 * r22z + r13x * obs.r33 * r32z;
    T r13_res = r11x * obs.r11 * r13z + r11x * obs.r12 * r23z + r11x * obs.r13 * r33z
              + r12x * obs.r21 * r13z + r12x * obs.r22 * r23z + r12x * obs.r23 * r33z
              + r13x * obs.r31 * r13z + r13x * obs.r32 * r23z + r13x * obs.r33 * r33z;

    T t1_res = r11x * obs.r11 * txz + r11x* obs.r12 * tyz + r11x * obs.r13 * tzz
              + r12x * obs.r21 * txz + r12x * obs.r22 * tyz + r12x * obs.r23 * tzz
              + r13x * obs.r31 * txz + r13x * obs.r32 * tyz + r13x * obs.r33 * tzz
              + r11x * obs.t1 + r12x * obs.t2 + r13x * obs.t3 + txx;

    
    T r21_res = r11x * obs.r11 * r21z + r11x * obs.r12 * r22z + r11x * obs.r13 * r23z
              + r12x * obs.r21 * r21z + r12x * obs.r22 * r22z + r12x * obs.r23 * r23z
              + r13x * obs.r31 * r21z + r13x * obs.r32 * r22z + r13x * obs.r33 * r23z;
    T r22_res = r11x * obs.r11 * r22z + r11x * obs.r12 * r23z + r11x * obs.r13 * r33z
              + r12x * obs.r21 * r22z + r12x * obs.r22 * r23z + r12x * obs.r23 * r33z
              + r13x * obs.r31 * r22z + r13x * obs.r32 * r23z + r13x * obs.r33 * r33z;
    T r23_res = r11x * obs.r11 * r23z + r11x * obs.r12 * r33z + r11x * obs.r13 * t3
              + r12x * obs.r21 * r23z + r12x * obs.r22 * r33z + r12x * obs.r23 * t3
              + r13x * obs.r31 * r23z + r13x * obs.r32 * r33z + r13x * obs.r33 * t3;
    T t2_res = r21x * obs.r11 * txz + r21x * obs.r12 * tyz + r21x * obs.r13 * tzz
              + r22x * obs.r21 * txz + r22x * obs.r22 * tyz + r22x * obs.r23 * tzz
              + r23x * obs.r31 * txz + r23x * obs.r32 * tyz + r23x * obs.r33 * tzz
              + r21x * obs.t1 + r22x * obs.t2 + r23x * obs.t3 + txy;          

    T r31_res = r31x * obs.r11 * r11z + r31x * obs.r12 * r21z + r31x * obs.r13 * r31z
              + r32x * obs.r21 * r11z + r32x * obs.r22 * r21z + r32x * obs.r23 * r31z
              + r33x * obs.r31 * r11z + r33x * obs.r32 * r21z + r33x * obs.r33 * r31z;
    T r32_res = r31x * obs.r11 * r12z + r31x * obs.r12 * r22z + r31x * obs.r13 * r23z
              + r32x * obs.r21 * r12z + r32x * obs.r22 * r22z + r32x * obs.r23 * r23z
              + r33x * obs.r31 * r12z + r33x * obs.r32 * r22z + r33x * obs.r33 * r23z;
    T r33_res = r31x * obs.r11 * r13z + r31x * obs.r12 * r23z + r31x * obs.r13 * r33z
              + r32x * obs.r21 * r13z + r32x * obs.r22 * r23z + r32x * obs.r23 * r33z
              + r33x * obs.r31 * r13z + r33x * obs.r32 * r23z + r33x * obs.r33 * r33z;
    
    T t3_res = r31x * obs.r11 * txz + r31x * obs.r12 * tyz + r31x * obs.r13 * tzz
              + r32x * obs.r21 * txz + r32x * obs.r22 * tyz + r32x * obs.r23 * tzz
              + r33x * obs.r31 * txz + r33x * obs.r32 * tyz + r33x * obs.r33 * tzz
              + r31x * obs.t1 + r32x * obs.t2 + r33x * obs.t3 + txz;
    
    T scale = r31_res * obs.x + r32_res * obs.y + t3_res;
    T u_proj = (obs.x*(obs.alpha * r11_res + obs.u0 * r31_res) + obs.y*(obs.alpha * r12_res + obs.u0 * r32_res) + obs.alpha * t1_res + obs.u0 * t3_res)/scale;
    T v_proj = (obs.x*(obs.beta * r21_res + obs.v0 * r31_res) + obs.y*(obs.beta * r22_res + obs.v0 * r32_res) + obs.beta * t2_res + obs.v0 * t3_res)/scale;
    
    residuals[0] = u_proj - T(obs.u);
    residuals[1] = v_proj - T(obs.v);
    
    return true;
  }

  Observation obs;
};


class AXZB
{
public:
  AXZB() = default;
  ~AXZB();

  void setParams(Eigen::Matrix3d& intrinsic, std::vector<Eigen::Matrix4d>& Tbhs, std::vector<std::vector<Eigen::Matrix4d>>& Tcos);
  void doCalib();
  Eigen::Matrix4d getCalibMat();


private:
  void estimateExtrinsic_();
  void optimize();

  std::vector<Eigen::Matrix4d>& Tbhs_;
  std::vector<std::vector<Eigen::Matrix4d>>& Tcos_;

  Eigen::Matrix3d intrinsic_;
  Eigen::Quaterniond qa_;
  Eigen::Matrix4d Q_;
  Eigen::Matrix4d W_;
  Eigen::Matrix4d C_;

  Eigen::Matrix4d Z_;
  Eigen::Matrix4d X_;
};

#endif // AXZB_HPP