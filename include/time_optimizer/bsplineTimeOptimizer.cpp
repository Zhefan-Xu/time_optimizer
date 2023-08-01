/*
*	File: bsplineTimeOptimizer.h
*	---------------
*   bspline time optimizer header
*/

#include <time_optimizer/bsplineTimeOptimizer.h>

namespace timeOptimizer{
	bsplineTimeOptimizer::bsplineTimeOptimizer(const ros::NodeHandle& nh) : nh_(nh){
		this->trajDivider_.reset(new trajDivider (this->nh_));
		this->timeOpt_.reset(new timeOptimizer ());
	}

	void bsplineTimeOptimizer::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->trajDivider_->setMap(map);
	}
	
	bool bsplineTimeOptimizer::optimize(trajPlanner::bspline traj, double vmax, double amax, double dt){
		// obtain position, velocity, acceleration data
		double linearReparamFactor = this->getLinearReparamFactor(traj, vmax, amax);
		trajPlanner::bspline trajVel = traj.getDerivative();
		trajPlanner::bspline trajAcc = trajVel.getDerivative();
		std::vector<Eigen::Vector3d> posData, velData, accData;
		std::vector<double> sampleTime;
		for (double t=0.0; t * linearReparamFactor <= traj.getDuration(); t+=dt){
			sampleTime.push_back(t);
			posData.push_back(traj.at(t * linearReparamFactor));
			velData.push_back(trajVel.at(t * linearReparamFactor));
			accData.push_back(trajAcc.at(t * linearReparamFactor));
		}

		// divide trajectory
		this->trajDivider_->setTrajectory(posData, sampleTime);
		std::vector<std::pair<double, double>> tInterval;
		std::vector<double> obstacleDist;
		this->trajDivider_->run(tInterval, obstacleDist);
		std::vector<Eigen::Vector4d> nearestObstacles;
		this->trajDivider_->getNearestObstacles(nearestObstacles);

		// run optimizer
		this->timeOpt_->setLimits(vmax, amax);
		this->timeOpt_->loadTrajectory(posData, velData, accData, dt);
		this->timeOpt_->loadTimeInterval(tInterval);
		this->timeOpt_->loadObstacles(nearestObstacles);

		bool success = this->timeOpt_->optimize();
	}

	void bsplineTimeOptimizer::getStates(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc){

	}

	double bsplineTimeOptimizer::getLinearReparamFactor(trajPlanner::bspline traj, double vmax, double amax){
		double trajMaxVel = 0.0;
		double trajMaxAcc = 0.0;
		trajPlanner::bspline trajVel = traj.getDerivative();
		trajPlanner::bspline trajAcc = trajVel.getDerivative();
		double t = 0.0;
		while (t <= traj.getDuration()){
			Eigen::Vector3d vel = trajVel.at(t);
			Eigen::Vector3d acc = trajAcc.at(t);
			if (vel.norm() > trajMaxVel){
				trajMaxVel = vel.norm();
			}
			if (acc.norm() > trajMaxAcc){
				trajMaxAcc = acc.norm();
			}
			t+=0.1;
		}
		double factorVel = vmax/trajMaxVel;
		double factorAcc = sqrt(amax/trajMaxAcc);
		return std::min(factorVel, factorAcc);		
	}

}