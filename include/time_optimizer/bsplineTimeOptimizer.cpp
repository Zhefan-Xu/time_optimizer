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
	
	void bsplineTimeOptimizer::optimize(trajPlanner::bspline traj, double vmax, double amax, double dt){
		this->traj_ = traj;
		// obtain position, velocity, acceleration data
		double linearReparamFactor = this->getLinearReparamFactor(traj, vmax, amax);
		this->linearReparamFactor_ = linearReparamFactor;
		trajPlanner::bspline trajVel = traj.getDerivative();
		trajPlanner::bspline trajAcc = trajVel.getDerivative();
		std::vector<Eigen::Vector3d> posData, velData, accData;
		std::vector<double> sampleTime;
		for (double t=0.0; t * linearReparamFactor <= traj.getDuration(); t+=dt){
			sampleTime.push_back(t);
			posData.push_back(traj.at(t * linearReparamFactor));
			velData.push_back(trajVel.at(t * linearReparamFactor) *  linearReparamFactor);
			accData.push_back(trajAcc.at(t * linearReparamFactor) * pow(linearReparamFactor, 2));
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

		this->optimizeSuccess_ = this->timeOpt_->optimize();
	}

	double bsplineTimeOptimizer::getStates(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc){
		if (this->optimizeSuccess_){
			double alpha, beta;
			double optimizedTime = this->timeOpt_->remapTime(t, alpha, beta);
			double trajTime = optimizedTime * this->linearReparamFactor_;
			pos = this->traj_.at(trajTime);
			Eigen::Vector3d velReparam = this->traj_.getDerivative().at(trajTime) * this->linearReparamFactor_;
			Eigen::Vector3d accReparam = this->traj_.getDerivative().getDerivative().at(trajTime) * pow(this->linearReparamFactor_, 2);
			vel = velReparam * sqrt(beta);
			acc = velReparam * alpha + accReparam * beta;
			// need further noramlize
			if (vel.norm() >= this->timeOpt_->getMaxVel()){
				vel *= this->timeOpt_->getMaxVel()/vel.norm();
			}
			if (acc.norm() >= this->timeOpt_->getMaxAcc()){
				acc *= this->timeOpt_->getMaxAcc()/acc.norm();
			}

			return trajTime;
		}
		else{
			double trajTime = t * this->linearReparamFactor_;
			pos = this->traj_.at(trajTime);
			vel = this->traj_.getDerivative().at(trajTime) * this->linearReparamFactor_;
			acc = this->traj_.getDerivative().getDerivative().at(trajTime) * pow(this->linearReparamFactor_, 2);
			return trajTime;			
		}
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

	double bsplineTimeOptimizer::getDuration(){
		if (this->optimizeSuccess_){
			return this->timeOpt_->getDuration();
		}
		else{
			return this->traj_.getDuration()/this->linearReparamFactor_;
		}
	}

}