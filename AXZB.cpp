#include "AXZB.hpp"

void AXZB::setParams(Eigen::Matrix3d& intrinsic, std::vector<Eigen::Matrix4d>& Tbhs, std::vector<std::vector<Eigen::Matrix4d>>& Tcos)
{
  intrinsic_ = intrinsic;
  Tbhs_ = Tbhs; // With number of images
  Tcos_ = Tcos; //
}

void AXZB::doCalib(){
  Eigen::Matrix4d C;
  C.setZero();
  int N = Tcos_.size();

  Eigen::MatrixXd A(0, 6);
  Eigen::Matrix3d Identity = -Eigen::Matrix3d::Identity();

  std::vector<Eigen::Vector3d> tbs;
  std::vector<Eigen::Vector3d> tas;

  for(int i = 0; i < N; i++){ // for each image
    int M = Tcos_.at(i).size(); 
    for (int j = 0; j < M; j++) // for each feature point
    {
      Eigen::Matrix4d Thb = inverseSE3(Tbhs_.at(i));
      Eigen::Matrix4d Tco = Tcos_.at(i).at(j);

      Eigen::Matrix3d Ra = Tco.block<3,3>(0,0);
      Eigen::Vector3d ta = Tco.block<3,1>(0,3);
      Eigen::Matrix3d Rb = Thb.block<3,3>(0,0);
      Eigen::Vector3d tb = Thb.block<3,1>(0,3);

      tbs.push_back(tb);
      tas.push_back(ta);

      int rows = A.rows();
      A.conservativeResize(rows + 3, Eigen::NoChange);
      A.block(rows , 0, 3, 3) = Ra;
      A.block(rows , 3, 3, 3) = Identity;

      Eigen::Quaterniond qa(Ra);
      Eigen::Quaterniond qb(Rb);

      Eigen::Matrix4d Q;
      Eigen::Matrix4d W;

      q2Q(qa, Q);
      q2W(qb, W);

      Eigen::Matrix4d Ci;

      Ci.setIdentity();
      Ci = -Q.transpose()*W;

      C += Ci;
    }
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(C.transpose()*C);
  Eigen::Vector4d alphas = es.eigenvalues();
  Eigen::Matrix4d V = es.eigenvectors();

  double lambda = 999999999999;

  std::vector<double> alphasVec;
  for(int i = 0; i < 4; i++){
    double alpha = alphas(i);
    double ans = N + std::sqrt(alpha);
    if (ans > 0){
      lambda = std::min(lambda, ans);
    }
    ans = N - std::sqrt(alpha);
    if (ans > 0){
      lambda = std::min(lambda, ans);
    }
  }

  double alpha = std::pow(lambda - N, 2);
  int idx = -1;

  for(int i = 0; i < alphas.size(); ++i){
    if (std::abs(alphas(i) - lambda) < 1e-6){
      idx = i;
      break;
    }
  }

  Eigen::Quaterniond qz;
  Eigen::Quaterniond qx;
  qz = V.col(idx);
  qx = (1/(lambda - N))*(C*qz);

  std::cout << "qz:\n" << qz.coeffs() << '\n';
  std::cout << "qx:\n" << qx.coeffs() << '\n';

  Eigen::Matrix3d Rz;
  Rz = qz.toRotationMatrix();
  Eigen::Matrix3d Rx;
  Rx = qx.toRotationMatrix();

  Eigen::MatrixXd B(0,1);
  for (int i = 0; i < A.rows() / 3; i++)
  {
    Eigen::Vector3d b = Rz * tbs.at(i) - tas.at(i);

    int rows = B.rows();
    B.conservativeResize(rows + 3, Eigen::NoChange);
    B.block(rows, 0, 3, 1) = b;
  }

  Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

  std::cout << "x:\n" << x.transpose() << '\n';

  Eigen::Vector3d tx = x.block<3,1>(0,0);
  Eigen::Vector3d tz = x.block<3,1>(3,0);

  Z_ = Eigen::Matrix4d::Identity();
  Z_.block<3,1>(0,3) = tz;
  Z_.block<3,3>(0,0) = Rz;

  X_ = Eigen::Matrix4d::Identity();
  X_.block<3,1>(0,3) = tx;
  X_.block<3,3>(0,0) = Rx;

  std::cout << "Z:\n" << Z_ << '\n';
  std::cout << "X:\n" << X_ << '\n';

  optimize();
}

void AXZB::optimize()
{
  std::cout << "optimize start!!\n";
  ceres::Solver::Options  options; // Ceres option
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Problem          problem;

  Eigen::Matrix3d Rx = inverseSE3(X_).block<3,3>(0,0);
  Eigen::Matrix3d Rz = Z_.block<3,3>(0,0);

  Eigen::Vector3d eularx = Rx.eulerAngles(0, 1, 2);
  Eigen::Vector3d eularz = Rz.eulerAngles(0, 1, 2);

  Eigen::Vector3d tx = inverseSE3(X_).block<3,1>(0,3);
  Eigen::Vector3d tz = Z_.block<3,1>(0,3);

  double* params = new double[12] {
    eularx(0), eularx(1), eularx(2),
    tx(0), tx(1), tx(2),
    eularz(0), eularz(1), eularz(2),
    tz(0), tz(1), tz(2)
  };

  int N = Tbhs_.size();
  int M = Tcos_.size();
  Eigen::Matrix<double, 3, 4> P_I0;
  P_I0 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0 ;

  double alpha = intrinsic_(0, 0);
  double beta  = intrinsic_(1, 1);
  double u0    = intrinsic_(0, 2);
  double v0    = intrinsic_(1, 2);

  for (int i = 0; i < N; i++)
  {
    for (int j = 0; j < M; j++){  // For each feature point
      Eigen::Matrix4d TcoIdeal = Tcos_.at(i).at(j);

      
      double wx = Tcos_.at(i).at(j).block<3,1>(0,3).transpose()(0);
      double wy = Tcos_.at(i).at(j).block<3,1>(0,3).transpose()(1);
      double wz = 0;
      double ws = 1;
      Eigen::Vector4d world(wx, wy, wz, ws);

      Eigen::Vector3d pixel = intrinsic_ * P_I0 * Tcos_.at(i).at(j) * world;
      
      std::cout << "pixel:\n" << pixel << '\n';

      double cs = pixel(2);
      double cu = pixel(0) / cs;
      double cv = pixel(1) / cs;

      std::unique_ptr<Observation> observed = std::make_unique<Observation>(wx, wy, cu, cv, alpha, beta, u0, v0);
      ceres::CostFunction* costFunction = 
        new ceres::AutoDiffCostFunction<ImageReprojectionError, 2, 12>(new ImageReprojectionError(*observed));
      problem.AddResidualBlock(costFunction, nullptr, params);
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double ax = params[0];
  double bx = params[1];
  double gx = params[2];
  double t1x = params[3];
  double t1x = params[4];
  double t1x = params[5];

  double az = params[6];
  double bz = params[7];
  double gz = params[8];
  double t2x = params[9];
  double t2x = params[10];
  double t2x = params[11];

  Eigen::Matrix3d Rx = Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                       Eigen::AngleAxisd(bx, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                       Eigen::AngleAxisd(gx, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d Rz = Eigen::AngleAxisd(az, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                       Eigen::AngleAxisd(bz, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                       Eigen::AngleAxisd(gz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d tx(t1x, t1x, t1x);
  Eigen::Vector3d tz(t2x, t2x, t2x);
  Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
  X.block<3,3>(0,0) = Rx;
  X.block<3,1>(0,3) = tx;
  Eigen::Matrix4d Z = Eigen::Matrix4d::Identity();
  Z.block<3,3>(0,0) = Rz;
  Z.block<3,1>(0,3) = tz;

  std::cout << "Optimization Summary: \n" << summary.FullReport() << "\n";
  std::cout << "Final params:\n";
  std::cout << "X:\n" << X << '\n';
  std::cout << "Z:\n" << Z << '\n';
}

