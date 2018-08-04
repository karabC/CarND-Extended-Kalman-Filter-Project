#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
  /**
  TODO:
    * predict the state
  */
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ *	Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd k = PHt * Si;
	x_ = x_ + (k*y);

	long xsize = x_.size();
	MatrixXd I = MatrixXd::Identity(xsize, xsize);
	P_ = (I - k * H_)*P_;
}

VectorXd KalmanFilter::PredictRadarMeasurement(const VectorXd& x) const
{
	VectorXd result(3);
	const float px = x(0, 0);
	const float py = x(1, 0);
	const float vx = x(2, 0);
	const float vy = x(3, 0);
	const float eps = 1e-8;
	const float rho = sqrt(px * px + py * py);
	const float phi = atan2(py, px);
	const float rho_dot = (px * vx + py * vy) / (eps + rho);
	result << rho, phi, rho_dot;
	return result;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd z_pred = PredictRadarMeasurement(x_);
	VectorXd y = z - z_pred;


	while (y[1] > M_PI || y[1] < -M_PI) {
		if (y[1] > M_PI) {
			y[1] = y[1] - 2 * M_PI;
		}
		else if (y[1] < -M_PI) {
			y[1] = y[1] + 2 * M_PI;
		}
	}

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_*Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd k = PHt * Si;

	long xsize = x_.size();
	MatrixXd I = MatrixXd::Identity(xsize, xsize);

	x_ = x_ + (k*y);

	P_ = (I - k * H_)*P_;
}
