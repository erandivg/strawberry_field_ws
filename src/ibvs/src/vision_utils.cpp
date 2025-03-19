/*

 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.

 */

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "vision_utils.h"
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <visp3/vision/vpHomography.h>



/*******************************************************************************
 *
 *                                functions
 *
 ******************************************************************************/


namespace {


    inline std::pair<FPTYPE, FPTYPE> unitize(FPTYPE x, FPTYPE y)
    {
        FPTYPE magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        FPTYPE l = x / magnitude;
        FPTYPE m = y / magnitude;
        return std::make_pair(l, m);
    }

    Vector3D_t vex(const Matrix_t &A) {
        if (A.rows()!=3 || A.cols()!=3)
            throw std::domain_error("vex: expects 3x3 matrices as an input");
        Vector3D_t v;
        v(0)=0.5*(A(2,1)-A(1,2));
        v(1)=0.5*(A(0,2)-A(2,0));
        v(2)=0.5*(A(1,0)-A(0,1));
        return v;
    }

}; // namespace


Matrix_t ComputeHomography( const std::vector<cv::Point2d> &cv_reference_points,
                            const std::vector<cv::Point2d> &cv_current_points,
                            const Matrix_t &K)
{
    cv::Mat cv_homography = cv::findHomography(cv_reference_points, cv_current_points);
    Matrix_t homography(3, 3);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            homography(i,j) = cv_homography.at<double>(i,j);
    return K.inverse()*homography*K;
}

Matrix_t ComputeEssential( const std::vector<cv::Point2d> &cv_reference_points,
                           const std::vector<cv::Point2d> &cv_current_points,
                           const Matrix_t &K)
{
    cv::Mat cv_fundamental_matrix =
        findFundamentalMat(cv_reference_points, cv_current_points, cv::FM_8POINT, 3, 0.99);
    Matrix_t fundamental(3, 3);
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            fundamental(i,j) = cv_fundamental_matrix.at<double>(i,j);
    return K.transpose()*fundamental*K;
}

void RecoverFromEssential(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t)
{
    // Decomposes the essential matrix E (3x3) into the camera motion
    // In practice there are multiple solutions and S (4x4xN) is a set of homogeneous
    // transformations representing possible camera motion.
    //
    // Reference::
    //
    // Y.Ma, J.Kosecka, S.Soatto, S.Sastry,
    // "An invitation to 3D",
    // Springer, 2003.
    // p116, p120-122
    // Notes::
    // - The transformation is from view 1 to view 2.
    // See also CentralCamera.E.
    // we return T from view 1 to view 2
    Matrix_t U;
    Matrix_t S;
    Matrix_t V;

    Eigen::JacobiSVD<Matrix_t> svd(homography, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    // Ma etal solution, p116, p120-122
    // Fig 5.2 (p113), is wrong, (R,t) is from camera 2 to 1
    if (V.determinant() < 0) {
        V = -V;
        S = -S;
    }
    if (U.determinant() < 0) {
        U = -U;
        S = -S;
    }
    static Eigen::Matrix3d mp(Eigen::AngleAxisd(+0.5*M_PI, Eigen::Vector3d::UnitZ()));
    static Eigen::Matrix3d mm(Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()));
    Matrix_t R1 = U*mp.transpose()*V.transpose();
    Matrix_t R2 = U*mm.transpose()*V.transpose();
    Vector3D_t t1 = vex(U*mp*S.asDiagonal()*U.transpose());
    Vector3D_t t2 = vex(U*mm*S.asDiagonal()*U.transpose());   
    if (R1(0,0)>0&&R1(1,1)>0&&R1(2,2)>0) 
       R=R1;
    else
       R=R2;
    if (t1(2)>0) // Suppose here that the target camera position is in front of us, not behind 
       t=t1;
    else
       t=t2;
}

void RecoverFromHomography(const Matrix_t &homography,Matrix_t &R, Vector3D_t &t, Vector3D_t &n, FPTYPE &distancePlane, const int current_iteration, homographySolution &homograpy_solution)
{
    Matrix_t U;
    Matrix_t S;
    Matrix_t V;

    {
        Eigen::JacobiSVD<Matrix_t> svd(homography, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        S = svd.singularValues();
        V = svd.matrixV();
    }

    // Eigen returns the transpose of what we need here
    V.transposeInPlace();

    FPTYPE s1 = S(0) / S(1);
    FPTYPE s3 = S(2) / S(1);
    FPTYPE zeta = s1 - s3;
    if (fabs(zeta)<std::numeric_limits<FPTYPE>::epsilon()) {
        distancePlane = 1.0;
        R = Matrix_t::Identity(3,3);
        n = Vector3D_t::Zero(3); n(0)=1.0;
        t = Vector3D_t::Zero(3); t(0)=1.0;
        return;
    }
    FPTYPE a1 = std::sqrt(1 - std::pow(s3, 2));
    FPTYPE b1 = std::sqrt(std::pow(s1, 2) - 1);
    std::pair<FPTYPE,FPTYPE> p,q,r;
    p = unitize(a1, b1);               FPTYPE &a = p.first;  FPTYPE &b = p.second;
    q = unitize(1 + s1 * s3, a1 * b1); FPTYPE &c = q.first;  FPTYPE &d = q.second;
    r = unitize(-b / s1, -a / s3);     FPTYPE &e = r.first;  FPTYPE &f = r.second;

    Matrix_t v1 = V.row(0).transpose();
    Matrix_t v3 = V.row(2).transpose();

    Vector3D_t n1 = b * v1 - a * v3;
    Vector3D_t n2 = b * v1 + a * v3;

    Matrix_t tmp(3, 3);
    tmp << c, 0.0, d,
           0, 1.0, 0,
          -d, 0.0, c;

    Matrix_t R1 = U * (tmp * V);
    Matrix_t R2 = U * (tmp.transpose() * V);

    Vector3D_t t1 = e * v1 + f * v3;
    Vector3D_t t2 = e * v1 - f * v3;


    if (Z(n1) < 0) {
        t1 = -t1;
        n1 = -n1;
    }

    if (Z(n2) < 0) {
        t2 = -t2;
        n2 = -n2;
    }
    if(current_iteration == 0) {
        if (Z(n1) > Z(n2)) { //Solution 1
            R = R1;
            t = t1;
            n = n1;
            homograpy_solution = homographySolution::SOLUTION_1;
        } else { //Solution 2
            R = R2;
            t = t1;
            n = n2;
            homograpy_solution = homographySolution::SOLUTION_2;
        }
    }else{
        if(homograpy_solution == homographySolution::SOLUTION_1){
            R = R1;
            t = t1;
            n = n1;
        }else{
            R = R2;
            t = t2;
            n = n2;
        }
    }

    distancePlane = 1.0 / zeta;
}


void EstimateHomography(const cv::Mat &refImg, const cv::Mat &currImg, const Matrix_t &K,  Matrix_t &H, const int counter )
{


    //*----------------- Step 1: Detect keypoints and compute keypoints using ORB Detector    -------------------------*//
    int nFeatures = 600;
    
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nFeatures);

    std::vector<cv::KeyPoint> keypointsRef, keypointsCurr;
    cv::Mat descriptorsRef, descriptorsCurr; 
    detector->detectAndCompute( refImg, noArray(), keypointsRef, descriptorsRef );
    detector->detectAndCompute( currImg, noArray(), keypointsCurr, descriptorsCurr ); 

    descriptorsRef.convertTo(descriptorsRef, CV_8U);
    descriptorsCurr.convertTo(descriptorsCurr, CV_8U);

    
    //*----------------- Step 2: Matching descriptor vectors with a FLANN based matcher  -------------------------*//
    //*----------------- Since ORB is a binary-point descriptor Hamming distance is used -------------------------*//

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptorsRef, descriptorsCurr, matches, 4);

    std::vector<cv::DMatch> goodMatches;
    for(const auto m : matches)
        if(m[0].distance < m[1].distance * 0.7f)
            goodMatches.push_back(m[0]);


    //*----------------- Step 3: Selecting good matches ans compute homography  -------------------------*//
    std::vector<Point2f> refPoints2f;
    std::vector<Point2f> currentPoints2f;

    for( size_t i = 0; i < goodMatches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        refPoints2f.push_back( keypointsRef[ goodMatches[i].queryIdx ].pt );
        currentPoints2f.push_back( keypointsCurr[ goodMatches[i].trainIdx ].pt );
    }

    // Verificar que hay al menos 4 puntos
    if (refPoints2f.size() < 4 || currentPoints2f.size() < 4) {
        std::cerr << "No hay suficientes puntos para calcular la homografía" << std::endl;
        return;
    }

    cv::Mat Hom = findHomography( refPoints2f, currentPoints2f, RANSAC, 1 );
    cv::cv2eigen(Hom,H);

    //std::cout << "H: " << H << std::endl;
    H = K.inverse()*H*K;
    H = H/H(1,1);
    //std::cout << "H: " << H << std::endl;

    // std::stringstream ss;
    // ss << "/home/noe/catkin_ws_tutorials/src/pbvs_controller2/src/folder_matches/matched_keypoints" << counter << ".jpg";

    // cv::Mat out;
    // cv::drawMatches(refImg, keypointsRef, currImg, keypointsCurr, goodMatches, out);
    // cv::imwrite(ss.str(),out);

    // cv::Mat out;
    // cv::drawKeypoints(currImg, keypointsCurr, out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    // cv::imwrite("/home/noe/catkin_ws_tutorials/src/pbvs_controller2/src/reference_image_keypoints.jpg",out);

    //std::cout <<"End of the EstimateHomography function... " << std::endl;

}

void EstimateHomographyVisp(const cv::Mat &refImg, const cv::Mat &currImg, const Matrix_t &K, vpHomography &H, const int counter )
{
    //*----------------- Step 1: Detect keypoints and compute keypoints using ORB Detector    -------------------------*//
    int nFeatures = 600;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nFeatures);
    std::vector<cv::KeyPoint> keypointsRef, keypointsCurr;
    cv::Mat descriptorsRef, descriptorsCurr;
    detector->detectAndCompute( refImg, noArray(), keypointsRef, descriptorsRef );
    detector->detectAndCompute( currImg, noArray(), keypointsCurr, descriptorsCurr );

    descriptorsRef.convertTo(descriptorsRef, CV_8U);
    descriptorsCurr.convertTo(descriptorsCurr, CV_8U);

    
    
    //*----------------- Step 2: Matching descriptor vectors with a FLANN based matcher  -------------------------*//
    //*----------------- Since ORB is a binary-point descriptor Hamming distance is used -------------------------*//

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptorsRef, descriptorsCurr, matches, 4);

    std::vector<cv::DMatch> goodMatches;
    for(const auto m : matches)
        if(m[0].distance < m[1].distance * 0.7f)
            goodMatches.push_back(m[0]);

    //*----------------- Step 3: Selecting good matches and compute homography  -------------------------*//
    
    std::vector<double> refPoints_x(goodMatches.size());
    std::vector<double> refPoints_y(goodMatches.size());
    std::vector<double>  currentPoints_x(goodMatches.size());
    std::vector<double>  currentPoints_y(goodMatches.size());

    Matrix_t Kinv_= K.inverse();

    //-- Get the keypoints from the good matches
   Point2D_t desnPoint, normPoint;
    for( size_t i = 0; i < goodMatches.size(); i++ )
    {
        //====================== Reference points ====================================
        desnPoint(0) =  keypointsRef[ goodMatches[i].queryIdx ].pt.x;
        desnPoint(1) =  keypointsRef[ goodMatches[i].queryIdx ].pt.y;

        //=================== points on the normalized plane =========================
        normPoint = NormalicePoint(desnPoint,Kinv_);

        refPoints_x[i] = normPoint(0);
        refPoints_y[i] = normPoint(1);

        //======================== current points ====================================
        desnPoint(0) =  keypointsCurr[ goodMatches[i].trainIdx ].pt.x;
        desnPoint(1) =  keypointsCurr[ goodMatches[i].trainIdx ].pt.y;
        normPoint = NormalicePoint(desnPoint,Kinv_);
        currentPoints_x[i] = normPoint(0);
        currentPoints_y[i] = normPoint(1);
    }

    vpHomography::HLM(currentPoints_x, currentPoints_y,refPoints_x, refPoints_y, false, H);
    H /= H[1][1];

}

void Rodriguez(const Matrix_t &R, Vector3D_t &u){

    cv::Mat Rcv, ucv;
    cv::eigen2cv(R,Rcv);
    cv::Rodrigues(Rcv,ucv);
    cv::cv2eigen(ucv,u);
}

void PBVSController(const Matrix_t &R, const Vector3D_t &t, const Vector3D_t &u, Vector3D_t &Uv,
                     Vector3D_t &Uw,const FPTYPE lambdav_, const FPTYPE lambdaw_)
{

    //*--------------- Linear velocity control --------------
    Uv = -lambdav_*R.transpose()*(t);
    //*--------------- Linear velocity control --------------

    //*--------------- Angular velocity control --------------
    Uw = -lambdaw_*u;
    //*--------------- Angular velocity control --------------

}

Point2D_t NormalicePoint(const Point2D_t &point, const Matrix_t &Kinv_ )
{
    
    Point3D_t normPoint = (Kinv_ * point.colwise().homogeneous());

    return normPoint.colwise().hnormalized();
}

void RecoverFromHomographyVisp(vpHomography &H, vpRotationMatrix &R, vpTranslationVector &t, vpColVector &n, vpThetaUVector &u)
{
    H.computeDisplacement(R,t,n);
    u.buildFrom(R);
}

void PBVSController(const vpRotationMatrix &R, const vpTranslationVector &t, const vpThetaUVector &u, 
                    vpColVector &Uv, vpColVector &Uw,const FPTYPE lambdav_,const FPTYPE lambdaw_)

{
        //*--------------- Linear velocity control --------------
    Uv = -lambdav_*R.t()*(t);
    //*--------------- Linear velocity control --------------

    //*--------------- Angular velocity control --------------
    Uw = -lambdaw_*u;
    //*--------------- Angular velocity control --------------

}
