/*
*	File: test_time_optimizer.cpp
*	---------------
*   time optimizer test
*/

#include <ros/ros.h>
#include <fstream>
#include <string>
#include <time_optimizer/timeOptimizer.h>
#include <trajectory_planner/bspline.h>

double linearFeasibilityReparam(trajPlanner::bspline traj, double vmax, double amax, double dt){
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
		t+=dt;
	}
	double factorVel = vmax/trajMaxVel;
	double factorAcc = sqrt(amax/trajMaxAcc);
	return std::min(factorVel, factorAcc);
}


int main(int argc, char**argv){
	ros::init(argc, argv, "test_time_optimizer_node");
	ros::NodeHandle nh;

	// read control points
	std::fstream controlPointFile;
	controlPointFile.open("../data/control_points_data/control_point_divide3.txt", std::ios::in);

	std::vector<Eigen::Vector3d> controlPointsVec;
	if (controlPointFile.is_open()){
		std::string line;
		while (getline(controlPointFile, line)){
			// cout << line << endl;
			Eigen::Vector3d p;
			std::istringstream s (line);
			double d;
			int i = 0;
			while (s >> d){
				p(i) = d;
				++i;
			}
			controlPointsVec.push_back(p);
		}
		controlPointFile.close();
	}

	// check control points
	Eigen::MatrixXd controlPoints;
	controlPoints.resize(3, int(controlPointsVec.size()));
	int i = 0;
	for (Eigen::Vector3d p : controlPointsVec){
		// cout << "ctl points: " << p.transpose() << endl;
		controlPoints.col(i) = p;
		++i;
	}


	// read time divide data
	std::fstream timeDivideFile;
	timeDivideFile.open("../data/divide_time_data/divide_time_3.txt", std::ios::in);

	std::vector<std::pair<double, double>> timeInterval;
	if (timeDivideFile.is_open()){
		std::string line;
		while (getline(timeDivideFile, line)){
			std::pair<double, double> interval;
			std::istringstream s (line);
			double d;
			s >> d;
			interval.first = d;
			s >> d;
			interval.second = d;
			timeInterval.push_back(interval);
		}
		timeDivideFile.close();
	}

	// check time interval
	// for (std::pair<double, double> interval : timeInterval){
	// 	cout << interval.first << " " << interval.second << endl;
	// }

	// read nearest obstacle data
	std::fstream nearestObstacleFile;
	nearestObstacleFile.open("../data/nearest_obstacle_data/nearest_obstacles3.txt", std::ios::in);

	std::vector<Eigen::Vector4d> nearestObstacles;
	if (nearestObstacleFile.is_open()){
		std::string line;
		while (getline(nearestObstacleFile, line)){
			Eigen::Vector4d ob;
			std::istringstream s (line);
			double d;
			int i = 0;
			while (s >> d){
				ob(i) = d;
				++i;
			}
			nearestObstacles.push_back(ob);
		}
		nearestObstacleFile.close();
	}

	// check nearest obstacle data
	// for (Eigen::Vector4d ob : nearestObstacles){
	// 	cout << "ob: " << ob.transpose() << endl;
	// }



	cout << "[Test]: Time optimizer." << endl; 

	double dt = 0.1;
	double vmax = 1.0;
	double amax = 3.0;
	trajPlanner::bspline traj (3, controlPoints, dt);
	double linearReparamFactor = linearFeasibilityReparam(traj, vmax, amax, dt);
	cout << "Linear reparam factor is: " << linearReparamFactor << endl;

	// obtain discrete position, velocity and acceleration data
	std::vector<Eigen::Vector3d> posData;
	std::vector<Eigen::Vector3d> velData;
	std::vector<Eigen::Vector3d> accData;
	double t = 0;
	while (t * linearReparamFactor <= traj.getDuration()){
		double trajTime = t * linearReparamFactor;
		Eigen::Vector3d pos = traj.at(trajTime);
		Eigen::Vector3d vel = traj.getDerivative().at(trajTime) * linearReparamFactor;
		Eigen::Vector3d acc = traj.getDerivative().getDerivative().at(trajTime) * pow(linearReparamFactor, 2);
		posData.push_back(pos);
		velData.push_back(vel);
		accData.push_back(acc);
		t += dt;
	}

	// for (int i=0; i<int(posData.size()); ++i){
	// 	cout << "pos: " << posData[i].transpose() << " , vel: " << velData[i].transpose() << ", acc: "  << accData[i].transpose() << endl;
	// }

	// velocity limits

	cout << "Total trajectory time is: " << t - dt << endl;

	ros::Time startTime = ros::Time::now();
	timeOptimizer::timeOptimizer opt;
	opt.setLimits(vmax, amax);
	opt.loadTrajectory(posData, velData, accData, dt);
	opt.loadTimeInterval(timeInterval);
	opt.optimize();
	ros::Time endTime = ros::Time::now();
	cout << "total time: " << (endTime - startTime).toSec() << endl;


	return 0;
}