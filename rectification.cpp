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
    extrinsic = rExtrinsic.at(i).cast<double>() * lExtrinsic.at(i).inverse().cast<double>(); //  Trg * Tlg^-1 == Trl
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
    Eigen::Matrix4d Tr = rExtrinsic_.at(imgIdx).cast<double>(); // Transform global --> right camera
    Eigen::Matrix4d Tl = lExtrinsic_.at(imgIdx).cast<double>();

    std::cout << "Tr:\n" << Tr << '\n';
    std::cout << "Tl:\n" << Tl << '\n';

    Eigen::Matrix3d Ar = rightParam_.block<3,3>(0,0);
    Eigen::Matrix3d Al = leftParam_.block<3,3>(0,0);

    Eigen::Matrix3d Rr = Tr.block<3,3>(0,0);
    Eigen::Matrix3d Rl = Tl.block<3,3>(0,0);

    Eigen::Vector3d tro = Tr.block<3,1>(0,3).transpose();
    Eigen::Vector3d tlo = Tl.block<3,1>(0,3).transpose();
    
    // calc optical center
    Eigen::Vector3d cr = -Rr.transpose() * Ar.inverse() * tro; // right camera's optical center in global frame
    Eigen::Vector3d cl = -Rl.transpose() * Al.inverse() * tlo; // left camera's optical center in global frame
    
    std::cout << "cr:\n" << cr << '\n';
    std::cout << "cl:\n" << cl << '\n';

    Eigen::Vector3d X = (cl - cr).normalized();                   // baseline
    Eigen::Vector3d Y = Tr.block<1,3>(2,0).cross(X).normalized(); // Tr's 3rd row means right camera's z axis reference to global frame
    Eigen::Vector3d Z = X.cross(Y).normalized();

    std::cout << "X:\n" << X << '\n';
    std::cout << "Y:\n" << Y << '\n';
    std::cout << "Z:\n" << Z << '\n';

    Eigen::Matrix3d Rinv;  // rotation converts new frame --> global
    
    Rinv << X, Y, Z;

    Eigen::Matrix3d R = Rinv.transpose();
    std::cout << "R:\n" << R << '\n';

    Eigen::Vector3d tr = -R * cr;
    Eigen::Vector3d tl = -R * cl;

    Eigen::Matrix3d K = rightParam_;

    Eigen::Matrix4d newTr;  // global --> rectified frame
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

    Eigen::Matrix<double, 3, 4> Pr = rightParam_ * P_I0 * Tr;
    Eigen::Matrix<double, 3, 4> Pl = leftParam_ * P_I0 * Tl;

    std::cout << "rightParam_:\n" << rightParam_ << '\n';
    std::cout << "leftParam_:\n" << leftParam_ << '\n';

    Eigen::Matrix3d newTrSE2 = newRproj.block<3,3>(0,0) * Pr.block<3,3>(0,0).inverse();  // SE(2) matrix
    Eigen::Matrix3d newTlSE2 = newLproj.block<3,3>(0,0) * Pl.block<3,3>(0,0).inverse();  // SE(2) matrix

    std::cout << "newTrSE2:\n" << newTrSE2 << '\n';
    std::cout << "newTlSE2:\n" << newTlSE2 << '\n';

    // newTrSE2 /= newTrSE2(2,2); // normalize
    // newTlSE2 /= newTlSE2(2,2); // normalize

    Eigen::Matrix2d rotationR = newTrSE2.block<2,2>(0,0);
    Eigen::Matrix2d rotationL = newTrSE2.block<2,2>(0,0);

    makeOrthogonal(rotationR);
    makeOrthogonal(rotationL);

    newTrSE2.block<2,2>(0,0) = rotationR;
    newTlSE2.block<2,2>(0,0) = rotationL;

    newTrSE2(2,0) = 0;
    newTrSE2(2,1) = 0;
    newTrSE2(2,2) = 1;
    newTlSE2(2,0) = 0;
    newTlSE2(2,1) = 0;
    newTlSE2(2,2) = 1;

    newTrs_.emplace_back(newTrSE2);
    newTls_.emplace_back(newTlSE2);

    std::cout << "newRproj:\n" << newRproj << '\n';
    std::cout << "newLproj:\n" << newLproj << '\n';
    std::cout << "Pr:\n" << Pr << '\n';
    std::cout << "Pl:\n" << Pl << '\n';
    std::cout << "new Tr:\n" << newTr << '\n';
    std::cout << "new Tl:\n" << newTl << '\n';
    std::cout << "new Tr SE(2):\n" << newTrSE2 << '\n';
    std::cout << "new Tl SE(2):\n" << newTlSE2 << '\n';
  }
}

void Rectification::visualize(std::vector<cv::Mat>& leftImages, std::vector<cv::Mat>& rightImages){
  for (int imgIdx = 0; imgIdx < Trls_.size(); imgIdx++)
  {
    cv::Mat leftImg   = leftImages.at(imgIdx);
    cv::Mat rightImg  = rightImages.at(imgIdx);

    Eigen::Matrix<double, 2, 3> Tr = newTrs_.at(imgIdx).block<2,3>(0,0);
    Eigen::Matrix<double, 2, 3> Tl = newTls_.at(imgIdx).block<2,3>(0,0);

    Eigen::Matrix<double, 3, 2> TrT = Tr.transpose();
    Eigen::Matrix<double, 3, 2> TlT = Tl.transpose();

    // Rectification matrices
    cv::Mat rightRect = cv::Mat(2, 3, CV_64F, TrT.data());
    cv::Mat leftRect  = cv::Mat(2, 3, CV_64F, TlT.data());

    // Apply rectification
    cv::Mat leftRectified, rightRectified;
    cv::warpAffine(rightImg, rightRectified, rightRect, rightImg.size());
    cv::warpAffine(leftImg, leftRectified, leftRect, leftImg.size());

    cv::Mat sideBySideOrg;
    cv::hconcat(leftImg, rightImg, sideBySideOrg);
    int numLines = 10;
    int stepOrg = sideBySideOrg.rows / numLines;
    for (int i = stepOrg; i < sideBySideOrg.rows; i += stepOrg){
      cv::line(sideBySideOrg, cv::Point(0, i), cv::Point(sideBySideOrg.cols, i), cv::Scalar(255,0,0), 1);
    }
    cv::imshow("Original Images (Left | Right)", sideBySideOrg);

    // Concatenate rectified images side-by-side
    cv::Mat sideBySide;
    cv::hconcat(leftRectified, rightRectified, sideBySide);

    // Draw horizontal epipolar lines
    int step = sideBySide.rows / numLines;
    for (int i = step; i < sideBySide.rows; i += step){
      cv::line(sideBySide, cv::Point(0, i), cv::Point(sideBySide.cols, i), cv::Scalar(0,255,0), 1);
    }

    // Display side-by-side rectified images with epipolar lines
    cv::imshow("Rectified Images (Left | Right)", sideBySide);
    cv::waitKey(0);
  }  
}