/*
*	File: bsplineTimeOptimizer.h
*	---------------
*   bspline time optimizer header
*/

#ifndef BSPLINE_TIME_OPTIMIZER_H
#define BSPLINE_TIME_OPTIMIZER_H
#include <memory>
#include <time_optimizer/timeOptimizer.h>

namespace timeOptimizer{
	class bsplineTimeOptimizer{
	private:
		double vmax_;
		double amax_;
		std::shared_ptr<timeOptimizer> timeOpt_;

	public:
		bsplineTimeOptimizer();
		void loadControlLimits(double vmax, double amax);
		void loadControlPoints(const Eigen::MatrixXd& controlPoints, double dt);
		bool optimize();
	};
}

#endif