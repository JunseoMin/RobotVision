#include "calibrationZhang.h"
/**
 * @brief calc intrinsic parameters with Zhang's method
 * @param lF Feature coord in camera pixel plane
 * @param mM 3D coord with Z = 0 (ignored)
 */
void KCalibrationZhang::doCalib(const std::vector<Eigen::MatrixXf>& lF, const Eigen::MatrixXf& mM)
{
  std::vector<Eigen::Vector3f> worldRaws; // Raw points from real-world coord (mM) (x,y)

  for (const auto& camMat : lF){
    std::vector<Eigen::Vector3f> camRaws;   // Raw points from camera pixels (lF) (u,v)
    Eigen::Vector3f pCam;
    Eigen::Vector3f pWorld;
    
    for (int i = 0; i < camMat.rows(); i ++){
      pCam.head<2>()    = camMat.row(i).transpose();
      pCam(2)           = 1.;
      camRaws.emplace_back(pCam);
      
      if (worldRaws.size() < mM.rows())
      {
        pWorld.head<2>()  = mM.row(i);
        pWorld(2)         = 1.;
        // std::cout << pWorld << '\n';
        worldRaws.emplace_back(pWorld);
        pWorld(2)         = 0.;
        worldPoints_.emplace_back(pWorld);  // save for optimization process
      }
     
    }
    // assert(worldRaws.size() == int(camRaws.size() / lF.size()));
    std::cout << "world size: " << worldRaws.size() << "\n";
    std::cout << "cam size: " << camRaws.size() << "\n";

    std::vector<Eigen::Vector3f> camNormal;

    camNormal = normalizePoint_(camRaws); // TODO: fix normalization code
    if (worldNormal_.size() == 0){  // normalize if not exists
      worldNormal_ = normalizePoint_(worldRaws, false);
    }
    
    std::cout << "Normalization finished!" << '\n';

    Eigen::Matrix3f H;  // Homograph matrix (3x3)
    H = calcHomography_(camNormal); // H -> unnormalized matrix
    std::cout << "H returned \n";
    homographies_.emplace_back(H);  // append homography matrix
    // current flow :  (T^-1) (T H) | T -> normalization matrix

    camPoints_.emplace_back(camRaws);
  }

  estimateIntrinsic_(); // calc intrinsic parameter
  estimateExtrinsic_();

  optimizeParams_();
}

/**
* @param raw (3x1) homogenious coordinate coordinates
*/
std::vector<Eigen::Vector3f> KCalibrationZhang::normalizePoint_(std::vector<Eigen::Vector3f> raw, bool isCam){
  assert(raw.size() > 0);
  
  int n = raw.size(); // Number of points
  std::vector<Eigen::Vector3f> normalized;
  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  

  for (const auto & p :raw){
    mean += p.head<2>();
  }
  mean /= n;

  float sumDist = 0.;
  for (const auto& p : raw){
    sumDist += (p.head<2>() - mean).norm();
  }

  float scale =  sqrt(2.f) / float(sumDist / n);
  std::cout << "scale value: " << scale << '\n';
  Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
  T(0,0) = scale;
  T(1,1) = scale;
  T(0,2) = -scale * mean(0);
  T(1,2) = -scale * mean(1);

  std::cout << "normalization matrix:\n" << T << '\n';

  for (const auto& p : raw){
    normalized.emplace_back(T * p);
  }

  if(isCam){
    TNormInv_ = Eigen::Matrix3f::Identity();
    TNormInv_ = T.inverse();
  }
  else{
    TNorm_ = Eigen::Matrix3f::Identity();
    TNorm_ = T;
  }

  return normalized;
}

Eigen::Matrix3f KCalibrationZhang::calcHomography_(std::vector<Eigen::Vector3f>& camNormal){
  int n = camNormal.size();
  
  // Q: I think this code is better.
  Eigen::MatrixXf A(2 * n, 9); 
  Eigen::Matrix3f H;

  for(int i = 0; i < n; i++){
    float x = worldNormal_[i](0);
    float y = worldNormal_[i](1);
    float u = camNormal[i](0);
    float v = camNormal[i](1);
    A.row(2*i + 0) <<    x,    y,   1,    0,   0,  0, -u*x, -u*y,  -u;
    A.row(2*i + 1) <<    0,    0,   0,    x,   y,  1, -v*x, -v*y,  -v;
  }
  // Result:
  // 651.375  -3.24059  322.714
  // 0  657.153  254.958
  // 0  0  1

  // From the lecture
  // Eigen::MatrixXf A(3 * n, 9);
  // Eigen::Matrix3f H;

  // for(int i = 0; i < n; i++){
  //   float x = worldNormal_[i](0);
  //   float y = worldNormal_[i](1);
  //   float u = camNormal[i](0);
  //   float v = camNormal[i](1);

  //   A.row(3*i + 0) <<    0,    0,   0,   -x,  -y,  1,  v*x,  v*y,   y;
  //   A.row(3*i + 1) <<    x,    y,   1,    0,   0,  0, -u*x, -u*y,  -u;
  //   A.row(3*i + 2) << -v*x, -v*y,  -v,  u*x, u*y,  u,    0,    0,   0;
  // }  // Results contains nan!!

  std::cout << "A matrix finished!" << "\n";
  
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
  Eigen::VectorXf h = svd.matrixV().col(8); // get last columns

  H << h(0), h(1), h(2),
       h(3), h(4), h(5),
       h(6), h(7), h(8);

  std::cout << "H matrix \n" << H << "\n";

  H = TNormInv_ * H * TNorm_;
  H /= H(2,2);

  std::cout << "H matrix \n" << H << "\n";

  return H;
}

void KCalibrationZhang::estimateIntrinsic_(){
  int n = homographies_.size();
  Eigen::MatrixXf V(2*n, 6);

  for(int i = 0; i < n; i++){
    const Eigen::Matrix3f& H = homographies_[i];

    float h11 = H(0,0), h12 = H(0,1), h13 = H(0,2);
    float h21 = H(1,0), h22 = H(1,1), h23 = H(1,2);
    float h31 = H(2,0), h32 = H(2,1), h33 = H(2,2);

    V.row(2*i + 0) << h11 * h12,
                      h11 * h22 + h21 * h12,
                      h21 * h22,
                      h31 * h12 + h11 * h32,
                      h31 * h22 + h21 * h32,
                      h31 * h32;
    
    Eigen::VectorXf v11(6), v22(6);

    v11 << h11 * h11,
           2 * h11 * h21,
           h21 * h21,
           2 * h11 * h31,
           2 * h21 * h31,
           h31 * h31;
    
    // v22:
    v22 << h12 * h12,
           2 * h12 * h22,
           h22 * h22,
           2 * h12 * h32,
           2 * h22 * h32,
           h32 * h32;

    V.row(2*i + 1) << (v11 - v22).transpose();
  }

  Eigen::JacobiSVD<Eigen::MatrixXf> svd(V, Eigen::ComputeFullV);
  std::cout << "Singular values of V: \n" << svd.singularValues().transpose() << "\n";
  Eigen::VectorXf B = svd.matrixV().col(svd.matrixV().cols() - 1);

  Eigen::Matrix3f Bmat;
  Bmat(0,0) = B(0);  // b11
  Bmat(0,1) = B(1);  // b12
  Bmat(0,2) = B(3);  // b13
  Bmat(1,0) = B(1);  // b12 
  Bmat(1,1) = B(2);  // b22
  Bmat(1,2) = B(4);  // b23
  Bmat(2,0) = B(3);  // b13 
  Bmat(2,1) = B(4);  // b23 
  Bmat(2,2) = B(5);  // b33
  
  float b11 = Bmat(0,0);
  float b12 = Bmat(0,1);
  float b13 = Bmat(0,2);
  float b22 = Bmat(1,1);
  float b23 = Bmat(1,2);
  float b33 = Bmat(2,2);
  
  float v0     = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
  float lambda = b33 - (b13 * b13 + v0 * (b12 * b13 - b11 * b23)) / b11;
  float alpha  = static_cast<float>(sqrt(lambda / b11));
  float beta   = static_cast<float>(sqrt(lambda * b11 / (b11 * b22 - b12 * b12)));
  float gamma  = (-b12 * alpha * alpha * beta) / lambda;
  float u0     = gamma * v0 / alpha - ((b13 * alpha * alpha) / lambda);
  

  std::cout << "B matrix: \n " << Bmat << '\n'; // TODO: B has a negative value
  std::cout << "lambda: " << lambda << "\n";
  
  A_ << alpha , 0     , u0,
        0     , beta  , v0,
        0     , 0     , 1;

  std::cout << "Intrinsic Parameter: \n" << A_ << "\n";
  
  camParamIdeal_.uo    = float(u0);
  camParamIdeal_.vo    = float(v0);
  camParamIdeal_.fu    = float(alpha);
  camParamIdeal_.fv    = float(beta);
  camParamIdeal_.gamma = float(gamma);
  camParamIdeal_.k1 = float(0.);
  camParamIdeal_.k2 = float(0.);
}

/**
 * @brief Estimate extrinsic parameter from matrix
 */
void KCalibrationZhang::estimateExtrinsic_() {
  for (const auto& H : homographies_) {
    Eigen::Matrix3f R;
    Eigen::Vector3f t;

    float lambda = 1.0 / (A_.inverse() * H.col(0)).norm();

    Eigen::Vector3f r1 = lambda * A_.inverse() * H.col(0);
    Eigen::Vector3f r2 = lambda * A_.inverse() * H.col(1);
    Eigen::Vector3f r3 = r1.cross(r2); // Ensure orthogonality
    t = lambda * A_.inverse() * H.col(2);

    R.col(0) = r1;
    R.col(1) = r2;
    R.col(2) = r3;
    
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV); // Orthogonalization
    R = svd.matrixU() * svd.matrixV().transpose();

    Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
    extrinsic.block<3, 3>(0, 0) = R;
    extrinsic.block<3, 1>(0, 3) = t;

    extrinsics_.push_back(extrinsic);

    std::cout << "Extrinsic param SE(3):\n" << extrinsic << '\n';
  }
}

/**
 * @brief Optimization with ceres solver
 */
void KCalibrationZhang::optimizeParams_(){
  std::cout << "optimization start!!\n";
  ceres::Solver::Options  options; // Ceres option 
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  ceres::Problem          problem;

  // Inital values to predict
  double k1 = 0.,k2 = 0.;
  double alpha  = static_cast<double>(camParamIdeal_.fu);
  double beta   = static_cast<double>(camParamIdeal_.fv);
  double u0     = static_cast<double>(camParamIdeal_.uo);
  double v0     = static_cast<double>(camParamIdeal_.vo);

  // Inital param array
  double* params = new double[6]{alpha, beta, u0, v0, k1, k2};

  for (int i = 0; i < camPoints_.size(); i++){
    for (int j = 0; j < camPoints_[i].size(); j++)
    {
      Eigen::Vector3f world;
      float x = worldPoints_[i](0);
      float y = worldPoints_[i](1);
      world << x , y , 1.f;

      Eigen::Vector3f idealCam = homographies_[i] * world;
      float cs = idealCam(2);
      float cu = idealCam(0) / cs;
      float cv = idealCam(1) / cs;

      std::unique_ptr<Observation> observed = std::make_unique<Observation>(
        double(cu), double(cv), double(x), double(y), 0.,  // (u, v , x, y, 0)
        static_cast<double>(extrinsics_[i](0,0)),static_cast<double>(extrinsics_[i](0,1)), static_cast<double>(extrinsics_[i](0,2)),static_cast<double>(extrinsics_[i](0,3)), // SE(3) values
        static_cast<double>(extrinsics_[i](1,0)),static_cast<double>(extrinsics_[i](1,1)), static_cast<double>(extrinsics_[i](1,2)),static_cast<double>(extrinsics_[i](1,3)), // SE(3) values
        static_cast<double>(extrinsics_[i](2,0)),static_cast<double>(extrinsics_[i](2,1)), static_cast<double>(extrinsics_[i](2,2)),static_cast<double>(extrinsics_[i](2,3))  // SE(3) values
      ); //cost function: u,v - u',v' and guess k1, k2, alpha, beta, u0, v0

      ceres::CostFunction* costFunction = 
        new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(new ReprojectionError(*observed));
      problem.AddResidualBlock(costFunction, nullptr, params);
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "Optimization Summary: \n" << summary.FullReport() << "\n";

  camParam_.fu = static_cast<float>(params[0]);
  camParam_.fv = static_cast<float>(params[1]);
  camParam_.uo = static_cast<float>(params[2]);
  camParam_.vo = static_cast<float>(params[3]);
  camParam_.k1 = static_cast<float>(params[4]);
  camParam_.k2 = static_cast<float>(params[5]);

  // logging final results
  std::cout << "Optimized Intrinsic Parameters: \n";
  std::cout << "focal length (fu, fv): (" << camParam_.fu << ", " << camParam_.fv << ")\n";
  std::cout << "principal point (u0, v0): (" << camParam_.uo << ", " << camParam_.vo << ")\n";
  std::cout << "distortion coefficients (k1, k2): (" << camParam_.k1 << ", " << camParam_.k2 << ")\n";
}

/**
 * @brief Calc std, mean and return
 */
void KCalibrationZhang::evalParamDiff() {
  float idealFu = camParamIdeal_.fu;
  float idealFv = camParamIdeal_.fv;
  float idealUo = camParamIdeal_.uo;
  float idealVo = camParamIdeal_.vo;
  float idealK1 = camParamIdeal_.k1;
  float idealK2 = camParamIdeal_.k2;
  
  float optFu = camParam_.fu;
  float optFv = camParam_.fv;
  float optUo = camParam_.uo;
  float optVo = camParam_.vo;
  float optK1 = camParam_.k1;
  float optK2 = camParam_.k2;

  float diffFu = optFu - idealFu;
  float diffFv = optFv - idealFv;
  float diffUo = optUo - idealUo;
  float diffVo = optVo - idealVo;

  float meanDiff = (diffFu + diffFv + diffUo + diffVo) / 4;
  float stdDiff = std::sqrt(
    (std::pow(diffFu - meanDiff, 2) + std::pow(diffFv - meanDiff, 2) +
     std::pow(diffUo - meanDiff, 2) + std::pow(diffVo - meanDiff, 2)) / 4
  );

  std::cout << "Ideal Parameters: " << '\n'
            << "  Fu: " << idealFu << ", Fv: " << idealFv << '\n'
            << "  Uo: " << idealUo << ", Vo: " << idealVo << '\n'
            << "  K1: " << idealK1 << ", K2: " << idealK2 << '\n';

  std::cout << "Optimized Parameters: " << '\n'
            << "  Fu: " << optFu << ", Fv: " << optFv << '\n'
            << "  Uo: " << optUo << ", Vo: " << optVo << '\n'
            << "  K1: " << optK1 << ", K2: " << optK2 << '\n';

  std::cout << "Parameter Differences: " << '\n'
            << "  Fu: " << diffFu << ", Fv: " << diffFv << '\n'
            << "  Uo: " << diffUo << ", Vo: " << diffVo << '\n';

  std::cout << "Mean Parameter Difference: " << meanDiff << '\n';
  std::cout << "Standard Deviation of Differences: " << stdDiff << '\n';

  meanParam_ = meanDiff;
  stdParam_ = stdDiff;
}

Eigen::Vector2f projectPoint(const Eigen::Vector3f& point3D, const KMONOCAM_PARAM& camParams) {
  float r2 = point3D(0) * point3D(0) + point3D(1) * point3D(1);
  float distortion = 1.0f + camParams.k1 * r2 + camParams.k2 * r2 * r2;

  float x = point3D(0) * distortion;
  float y = point3D(1) * distortion;

  float u = camParams.fu * x + camParams.uo;
  float v = camParams.fv * y + camParams.vo;

  return Eigen::Vector2f(u, v);
}

void KCalibrationZhang::evalCoordDiff(){
  double mean = 0.;
  double std = 0.;
  double n = worldPoints_.size();

  Eigen::Matrix3f A;  // Optimized camera matrix
  A << camParam_.fu, 0            , camParam_.uo,
       0           , camParam_.fv , camParam_.vo,
       0           , 0            , 1;
  
  for (int scene = 0; scene < camPoints_.size(); scene ++){
    for (int i = 0; i < worldPoints_.size(); i++){
      Eigen::Vector3f opti;
      Eigen::Vector3f world3;
      Eigen::Vector4f world4;

      float x = worldPoints_[i][0];
      float y = worldPoints_[i][1];
      world3 << x, y, 1.f;
      world4 << x, y, 0, 1.f;
      
      Eigen::Vector4f worldCoord  = extrinsics_[scene] * world4;
      
      float cz = worldCoord(2);
      float cx = worldCoord(0) / cz;
      float cy = worldCoord(1) / cz;

      Eigen::Vector3f tmp;
      tmp << cx , cy , 1.f;

      Eigen::Vector3f predCam     =  A * tmp;

      float ps = predCam(2);
      float pu = predCam(0) / ps;
      float pv = predCam(1) / ps;
      
      opti << pu, pv, 1;
      Eigen::Vector3f idealCam    = homographies_[scene] * world3;
      Eigen::Vector3f ideal;
      float is = idealCam(2);
      float iu = idealCam(0) / is;
      float iv = idealCam(1) / is;
      ideal << iu, iv, 1;

      double diff = (ideal - opti).norm();
      std += diff*diff;
      mean += diff;
    }
  }

  mean /= worldPoints_.size() * camPoints_.size();
  std  /= worldPoints_.size() * camPoints_.size();
  double rmse = std::sqrt(std);

  meanProj_ = float(mean);
  stdProj_ = float(rmse);
}

std::vector<float> KCalibrationZhang::getEval(){
  std::vector<float> eval;
  eval.emplace_back(meanParam_);
  eval.emplace_back(stdParam_);
  eval.emplace_back(meanProj_);
  eval.emplace_back(stdProj_);
  return eval;
}

/**
 * @brief returns 3x3 camera intrinsic parameter matrix (eigen3dx)
 * @param intrinsic 3x3 eigen matrix for camera intrinsic parameter
 * @param dist distortion values
 */
void KCalibrationZhang::getParam(Eigen::Matrix3d& intrinsic, std::vector<double>& dist){
  intrinsic = Eigen::Matrix3d::Identity();

  intrinsic(0,0) = double(camParam_.fu);
  intrinsic(1,1) = double(camParam_.fv);
  intrinsic(2,0) = double(camParam_.uo);
  intrinsic(2,1) = double(camParam_.vo);

  dist.emplace_back(camParam_.k1);
  dist.emplace_back(camParam_.k2);
}

void KCalibrationZhang::getExtrinsic(std::vector<Eigen::Matrix4f>& extrinsics){
  std::copy(extrinsics_.begin(), extrinsics_.end(), extrinsics.begin());
}