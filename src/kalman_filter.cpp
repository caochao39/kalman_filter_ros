#include <kalman_filter.h>
#include <stdexcept>

KalmanFilter::KalmanFilter(double P, double Q, double R):P_minus(P), Q(Q), R(R), initialized(false){}

KalmanFilter::KalmanFilter(){}

void KalmanFilter::init()
{
	x_hat = 0; 
	P = P_minus;
	initialized = true;
}

void KalmanFilter::init(const double x0)
{
	x_hat = x0;
	P = P_minus;
	initialized = true;
}

void KalmanFilter::update(const double z)
{
	if(!initialized)
		throw std::runtime_error("Filter is not initialized!");
	x_hat_minus = x_hat;
	P_minus = P + Q;
	K = P_minus / (P_minus + R);
	x_hat = x_hat_minus + K * (z - x_hat_minus);
	P = (1 - K) * P_minus;	
}

