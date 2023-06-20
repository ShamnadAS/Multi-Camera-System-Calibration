#pragma once
#include <vector>
#include <Eigen/Dense>

class CompareCameras
{
public:
	std::vector<Eigen::MatrixXd> ExtrinsicCamera1;
	std::vector<Eigen::MatrixXd> ExtrinsicCamera2;

	std::vector<Eigen::MatrixXd> RelativeOrientation;
	std::vector<Eigen::Vector3d> RelativePosition;

	void CalculateRelativePosition();
	void CalculateRelativeOrientation();
};
