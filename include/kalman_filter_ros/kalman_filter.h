#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter{

public:
	KalmanFilter();
  	KalmanFilter(double P, double Q, double R);
  	void init();
  	void init(double x0);
  	void update(const double z);
  	double state(){return x_hat;};
private:
	double P;
	double P_minus;
	double Q;
	double R;
	double x_hat;
	double x_hat_minus;
	double K;
	bool initialized;
};

#endif