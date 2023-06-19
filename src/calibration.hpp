#pragma once
#include<Eigen/Dense>

using Eigen::MatrixXd;

class Calibration
{
public:
	MatrixXd CalculateProjection(Eigen::Vector3d worldPos[], Eigen::Vector2d pixelPos[]);
	MatrixXd CalculateExtrinsicProperties(MatrixXd &projection);
};

MatrixXd Calibration::CalculateProjection(Eigen::Vector3d worldPos[], Eigen::Vector2d pixelPos[])
{
	unsigned int n = worldPos->count();

	MatrixXd M(2 * n, 12);
	M.setZero();

	for (int i = 0; i < n; i++)
	{
		M(2 * i, 0) = worldPos[i].x();
		M(2 * i, 1) = worldPos[i].y();
		M(2 * i, 2) = worldPos[i].z();
		M(2 * i, 3) = 1;

		M(2 * i, 8) = -worldPos[i].x() * pixelPos[i].x();
		M(2 * i, 9) = -worldPos[i].x() * pixelPos[i].x();
		M(2 * i, 10) = -worldPos[i].x() * pixelPos[i].x();
		M(2 * i, 11) = -pixelPos[i].x();

		M(2 * i + 1, 4) = worldPos[i].x();
		M(2 * i + 1, 5) = worldPos[i].x();
		M(2 * i + 1, 6) = worldPos[i].x();
		M(2 * i + 1, 7) = 1;

		M(2 * i + 1, 8) = -worldPos[i].y() * pixelPos[i].y();
		M(2 * i + 1, 9) = -worldPos[i].y() * pixelPos[i].y();
		M(2 * i + 1, 10) = -worldPos[i].y() * pixelPos[i].y();
		M(2 * i + 1, 11) = -pixelPos[i].y();
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

	return projection;
}

Eigen::RowVectorXd CreateVectorV(unsigned i, unsigned j, MatrixXd &projection)
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

MatrixXd Calibration::CalculateExtrinsicProperties(MatrixXd& projection)
{
	Eigen::MatrixXd V(2, 6);

	Eigen::RowVectorXd v00 = CreateVectorV(0, 0, projection);
	Eigen::RowVectorXd v01 = CreateVectorV(0, 1, projection);
	Eigen::RowVectorXd v11 = CreateVectorV(1, 1, projection);

	V.row(0) = v01;
	V.row(1) = v00 - v11;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullV);
	Eigen::VectorXd b = svd.matrixV().col(5);

	float alpha = sqrt(1 / b[0]);
	float beta = sqrt(1 / b[2]);
	float u = abs(b[3] / b[0]);
	float v = abs(b[4] / b[2]);

	Eigen::Matrix3d intrinsicMatrix;
	intrinsicMatrix << alpha  , 0,    u,
				 0	    , beta, v,
				 0      , 0,    1;

	Eigen::Matrix3d invIntrinsicMatrix = intrinsicMatrix.inverse();
	Eigen::Vector3d s = invIntrinsicMatrix * projection.col(0);
	double lambda = 1 / s.norm();

	MatrixXd extrinsicMatrix(3, 4);

	extrinsicMatrix.col(0) = lambda * invIntrinsicMatrix * projection.col(0);
	extrinsicMatrix.col(1) = lambda * invIntrinsicMatrix * projection.col(1);
	extrinsicMatrix.col(2) = lambda * invIntrinsicMatrix * projection.col(2);
	extrinsicMatrix.col(3) = lambda * invIntrinsicMatrix * projection.col(3);

	std::cout << intrinsicMatrix << std::endl;
	return extrinsicMatrix;
}
