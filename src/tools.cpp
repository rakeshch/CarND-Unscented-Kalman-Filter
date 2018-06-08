#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	//check for valid inputs by making sure:
	//estimation vector size should not be zero
	//estmation vector size should equal to grounf truth vector size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size())
	{
		cout << "Invalid estimations or ground truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
	//calculating mean
	rmse = rmse / estimations.size();

	//calculating the squared root
	rmse = sqrt(rmse.array());

	return rmse;
}