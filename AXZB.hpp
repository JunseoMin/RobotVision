#ifndef AXZB_HPP
#define AXZB_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>

class AXZB
{
public:
  AXZB() = default;
  ~AXZB();

private:
  Eigen::Quaterniond qa_;
  Eigen::Matrix4d Q_;
  Eigen::Matrix4d W_;
};

#endif // AXZB_HPP