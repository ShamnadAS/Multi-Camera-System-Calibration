#include <iostream>
#include <Eigen/Dense>
#include "src/calibrateCamera.hpp"
#include "src/compareCameras.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using Eigen::Vector3d;
using Eigen::Vector2d;

int main()
{
    CalibrateCamera calibrateCamera1;
    CalibrateCamera calibrateCamera2;
	
    std::vector<Eigen::MatrixXd> extrinsicCamera1;
    std::vector<Eigen::MatrixXd> extrinsicCamera2;

    calibrateCamera1.CalculateResult("src/images/Camera 1/*.jpg", cv::Size{ 9, 6 }, extrinsicCamera1);
    calibrateCamera1.CalculateResult("src/images/Camera 2/*.jpg", cv::Size{ 9, 6 }, extrinsicCamera2);
    
    std::vector<Eigen::Vector3d> relativePosWtrCamera1;
    std::vector<Eigen::Matrix3d> relativeOrientationWtrCamera1;
    CompareCameras compareCameras(extrinsicCamera1, extrinsicCamera2);

    compareCameras.CalculateRelativeWtrCamera1(relativePosWtrCamera1, relativeOrientationWtrCamera1);
}
