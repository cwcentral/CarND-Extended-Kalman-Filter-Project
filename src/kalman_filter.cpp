/////////////////////////////////////////
#include "kalman_filter.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
		MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

#define M_PI 3.14159265359
#define M_2PI (2 * M_PI)

float constrain_pi(float in1)
{
	 if (in1 > M_PI)
	        in1 -= M_2PI;
	 else if (in1 < -M_PI)
	        in1 += M_2PI;

	 return in1;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	 TODO:
	 * update the state by using Extended Kalman Filter equations
	 */
	float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float phi = 0.;
	if (x_(0) != 0 && x_(1) != 0)
			phi = atan2(x_(1) , x_(0));

	std::cout << "******************************* " << rho<< "::" << phi << std::endl;

	float rho_dot = 0.;
	if (rho < 0.0001)
		rho_dot = 0.;
	else
		rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;

	y(1) = constrain_pi(y(1));

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
