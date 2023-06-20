#pragma once
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>

using Eigen::MatrixXd;

class CalibrateCamera
{
public:
	void CalculateResult(const std::string imagesRoot, const cv::Size& gridSize);

private:
	bool display = true;
	std::vector<std::vector<cv::Point2f>> BoardPts;
	std::vector<std::vector<cv::Point2f>> ImagePts;
	void CalculateProjection(std::vector<Eigen::MatrixXd> &projectionMatrices);
	void CalculateExtrinsicProperties(std::vector<Eigen::MatrixXd>& cameraExtrinsics, std::vector<Eigen::MatrixXd>& projectionMatrices);
	void HandleInputs(const std::string imagesRoot, const cv::Size& gridSize);
	void DisplayInputs(const cv::Mat& image, std::vector<cv::Point2f> pts, cv::Size gridSize, bool isFound);
};

void CalibrateCamera::CalculateResult(const std::string imagesRoot, const cv::Size& gridSize)
{
	std::vector<Eigen::MatrixXd> projectionMatrices;

	this->HandleInputs(imagesRoot, gridSize);
	this->CalculateProjection(projectionMatrices);
	std::cout << projectionMatrices[0] << std::endl;
	//this->CalculateExtrinsicProperties(extrinsics, projectionMatrices);
}

void CalibrateCamera::HandleInputs(const std::string imagesRoot, const cv::Size& gridSize)
{
	std::vector<cv::Mat> images;

	std::vector<cv::String> imagesPath;
	cv::glob(imagesRoot, imagesPath);
	for (const auto& path : imagesPath)
	{
		cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
		images.push_back(img);
	}

	ImagePts.clear();
	BoardPts.clear();
	for (const auto& img : images)
	{
		if (1 != img.channels())
		{
			continue;
		}
		std::vector<cv::Point2f> cornerPts;
		bool isFound = cv::findChessboardCorners(img, gridSize, cornerPts,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (!isFound)
		{
			continue;
		}
		cv::TermCriteria criteria(2, 30, 0.001);
		cv::cornerSubPix(img, cornerPts, gridSize, cv::Size(-1, -1), criteria);

		ImagePts.push_back(cornerPts);

		//setup board points
		std::vector<cv::Point2f> pts;

		for (int r = 0; r < gridSize.height; ++r)
		{
			for (int c = 0; c < gridSize.width; ++c)
			{
				pts.emplace_back(c, -r);
			}
		}
		BoardPts.push_back(pts);

		if(display)
			this->DisplayInputs(img, cornerPts, gridSize, isFound);
	}
}

void CalibrateCamera::DisplayInputs(const cv::Mat& image, std::vector<cv::Point2f> cornerPts, cv::Size gridSize, bool isFound)
{
	cv::Mat frame;
	if (1 == image.channels())
	{
		cv::cvtColor(image, frame, cv::COLOR_GRAY2BGR);
	}
	else
	{
		image.copyTo(frame);
	}

	cv::drawChessboardCorners(frame, gridSize, cornerPts, isFound);
	cv::imshow("Image", frame);
	cv::waitKey(0);
}

void  CalibrateCamera::CalculateProjection(std::vector<Eigen::MatrixXd> &projectionMatrices)
{
	for (int i = 0;  i < BoardPts.size(); i++)
	{
		unsigned n = BoardPts[i].size();
		
		MatrixXd M(2 * n, 12);
		M.setZero();

		for (int j = 0; j < n; j++)
		{
			cv::Point2f boardPoint = BoardPts[i][j];
			cv::Point2f imagePoint = ImagePts[i][j];

			M(2 * j, 0) = boardPoint.x;
			M(2 * j, 1) = boardPoint.y;
			M(2 * j, 2) = 0;
			M(2 * j, 3) = 1;

			M(2 * j, 8) = -boardPoint.x * imagePoint.x;
			M(2 * j, 9) = -boardPoint.y * imagePoint.x;
			M(2 * j, 10) = 0;
			M(2 * j, 11) = -imagePoint.x;

			M((2 * j) + 1, 4) = boardPoint.x;
			M((2 * j) + 1, 5) = boardPoint.y;
			M((2 * j) + 1, 6) = 0;
			M((2 * j) + 1, 7) = 1;

			M((2 * j) + 1, 8) = -boardPoint.x * imagePoint.x;
			M((2 * j) + 1, 9) = -boardPoint.y * imagePoint.y;
			M((2 * j) + 1, 10) = 0;
			M((2 * j) + 1, 11) = -imagePoint.y;
		}

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
		Eigen::VectorXd V = svd.matrixV().col(11);

		MatrixXd projection(3, 4);

		projection(0, 0) = V(0);
		projection(0, 1) = V(1);
		projection(0, 2) = V(2);
		projection(0, 3) = V(3);

		projection(1, 0) = V(4);
		projection(1, 1) = V(5);
		projection(1, 2) = V(6);
		projection(1, 3) = V(7);

		projection(2, 0) = V(8);
		projection(2, 1) = V(9);
		projection(2, 2) = V(10);
		projection(2, 3) = V(11);

		projectionMatrices.push_back(projection);
	}
}

Eigen::RowVectorXd CreateVectorV(unsigned i, unsigned j, MatrixXd& projection)
{
	Eigen::RowVectorXd v(6);

	v(0) = projection(0, i) * projection(0, j);
	v(1) = (projection(0, i) * projection(1, j)) + (projection(1, i) * projection(0, j));
	v(2) = projection(1, i) * projection(1, j);
	v(3) = (projection(2, i) * projection(0, j)) + (projection(0, i) * projection(2, j));
	v(4) = (projection(2, i) * projection(1, j)) + (projection(1, i) * projection(2, j));
	v(5) = projection(2, i) * projection(2, j);

	return v;
}

void CalibrateCamera::CalculateExtrinsicProperties(std::vector<Eigen::MatrixXd> &cameraExtrinsics, 
	std::vector<Eigen::MatrixXd> &projectionMatrices)
{
	//calculate intrinsic
	unsigned n = projectionMatrices.size();

	Eigen::MatrixXd V(2 * n, 6);

	for (int i = 0; i < n; i++)
	{
		Eigen::RowVectorXd v00 = CreateVectorV(0, 0, projectionMatrices[i]);
		Eigen::RowVectorXd v01 = CreateVectorV(0, 1, projectionMatrices[i]);
		Eigen::RowVectorXd v11 = CreateVectorV(1, 1, projectionMatrices[i]);

		V.row(2 * n) = v01;
		V.row((2 * n) + 1) = v00 - v11;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);
	Eigen::VectorXd b = svd.matrixV().col(5);

	float alpha = sqrt(1 / b[0]);
	float beta = sqrt(1 / b[2]);
	float u = abs(b[3] / b[0]);
	float v = abs(b[4] / b[2]);

	Eigen::Matrix3d intrinsicMatrix;
	intrinsicMatrix << alpha, 0, u,
		0, beta, v,
		0, 0, 1;

	//calculate extrinsic
	Eigen::Matrix3d invIntrinsicMatrix = intrinsicMatrix.inverse();

	for (int i = 0; i < n; i++)
	{
		Eigen::Vector3d s = invIntrinsicMatrix * projectionMatrices[i].col(0);
		double lambda = 1 / s.norm();

		MatrixXd extrinsicMatrix(3, 4);

		extrinsicMatrix.col(0) = lambda * invIntrinsicMatrix * projectionMatrices[i].col(0);
		extrinsicMatrix.col(1) = lambda * invIntrinsicMatrix * projectionMatrices[i].col(1);
		extrinsicMatrix.col(2) = lambda * invIntrinsicMatrix * projectionMatrices[i].col(2);
		extrinsicMatrix.col(3) = lambda * invIntrinsicMatrix * projectionMatrices[i].col(3);

		cameraExtrinsics.push_back(extrinsicMatrix);
	}
}
