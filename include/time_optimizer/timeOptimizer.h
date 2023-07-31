/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer header file
*/


#ifndef TIME_OPTIMIZER_H
#define TIME_OPTIMIZER_H

#include <iostream>
#include <Eigen/Eigen>
#include <mosek.h>
using std::cout; using std::endl;

static void MSKAPI printstr(void *handle,
                            const char str[]){
  	printf("%s", str);
} 

namespace timeOptimizer{
	class timeOptimizer{
	private:
		double lambda_ = 0.5;
		double vmax_;
		double amax_;
		double dt_;
		std::vector<Eigen::Vector3d> posData_;
		std::vector<Eigen::Vector3d> velData_;
		std::vector<Eigen::Vector3d> accData_;
		std::vector<std::pair<double, double>> timeInterval_;

		std::vector<std::vector<Eigen::Vector3d>> posDataList_;
		std::vector<std::vector<Eigen::Vector3d>> velDataList_;
		std::vector<std::vector<Eigen::Vector3d>> accDataList_;

		std::vector<double> trajTime_;
		std::vector<double> realTime_;
		std::vector<double> alphaSol_;
		std::vector<double> betaSol_;

	public:
		timeOptimizer();
		void setLimits(double vmax, double amax);
		void loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							double dt);
		void loadTimeInterval(const std::vector<std::pair<double, double>>& timeInterval);
		void divideData(std::vector<std::vector<Eigen::Vector3d>>& posDataList, 
						std::vector<std::vector<Eigen::Vector3d>>& velDataList,
						std::vector<std::vector<Eigen::Vector3d>>& accDataList,
						std::vector<bool>& obstacleInfoList);
		bool optimize();
		void extractSol(const std::vector<double>& sol);
		double remapTime(double tau, double& alpha, double& beta);
	};
}

#endif