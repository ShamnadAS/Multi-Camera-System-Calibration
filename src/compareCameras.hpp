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

	CompareCameras(std::vector<Eigen::MatrixXd>& extrinsicCamera1, std::vector<Eigen::MatrixXd>& extrinsicCamera2);
	void CalculateRelativeWtrCamera1(std::vector<Eigen::Vector3d>& relativePosWtrCamera1,
		std::vector<Eigen::Matrix3d>& relativeOrientationWtrCamera1);

private:
	void CalculateRelativePosition(std::vector<Eigen::Vector3d>& relativePosWtrCamera1);
	void CalculateRelativeOrientation(std::vector<Eigen::Matrix3d>& relativeOrientationWtrCamera1);
};

void CompareCameras::CalculateRelativePosition(std::vector<Eigen::Vector3d>& relativePosWtrCamera1)
{
	int n = ExtrinsicCamera2.size();

	for (int i = 0; i < n; i++)
	{
		Eigen::Vector3d posCamera2 = ExtrinsicCamera2[i].col(3).transpose();
		Eigen::Matrix3d orientationCamera1;
		orientationCamera1.col(0) = ExtrinsicCamera1[i].col(0);
		orientationCamera1.col(1) = ExtrinsicCamera1[i].col(1);
		orientationCamera1.col(2) = ExtrinsicCamera1[i].col(2);

		Eigen::Vector3d relativePos = orientationCamera1 * posCamera2;
		relativePosWtrCamera1.push_back(relativePos);
	}
}

void CompareCameras::CalculateRelativeOrientation(std::vector<Eigen::Matrix3d>& relativeOrientationWtrCamera1)
{
	int n = ExtrinsicCamera2.size();

	for (int i = 0; i < n; i++)
	{
		Eigen::Matrix3d orientationCamera1;
		orientationCamera1.col(0) = ExtrinsicCamera1[i].col(0);
		orientationCamera1.col(1) = ExtrinsicCamera1[i].col(1);
		orientationCamera1.col(2) = ExtrinsicCamera1[i].col(2);

		Eigen::Matrix3d orientationCamera2;
		orientationCamera2.col(0) = ExtrinsicCamera2[i].col(0);
		orientationCamera2.col(1) = ExtrinsicCamera2[i].col(1);
		orientationCamera2.col(2) = ExtrinsicCamera2[i].col(2);

		Eigen::Matrix3d relativeOrientation = orientationCamera1 * orientationCamera2;

		relativeOrientationWtrCamera1.push_back(relativeOrientation);
	}
}

void CompareCameras::CalculateRelativeWtrCamera1(std::vector<Eigen::Vector3d>& relativePosWtrCamera1,
	std::vector<Eigen::Matrix3d>& relativeOrientationWtrCamera1)
{
	CalculateRelativePosition(relativePosWtrCamera1);
	CalculateRelativeOrientation(relativeOrientationWtrCamera1);
}

CompareCameras::CompareCameras(std::vector<Eigen::MatrixXd>& extrinsicCamera1, std::vector<Eigen::MatrixXd>& extrinsicCamera2)
	:ExtrinsicCamera1(extrinsicCamera1), ExtrinsicCamera2(extrinsicCamera2)
{
}

