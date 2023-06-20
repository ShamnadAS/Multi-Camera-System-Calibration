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
	
    //std::vector<Eigen::MatrixXd> extrinsicsCamera1;
    calibrateCamera1.CalculateResult("src/images/*.jpg", cv::Size{ 9, 6 });
    //std::cout << extrinsicsCamera1[0] << std::endl;
}
