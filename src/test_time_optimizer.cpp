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

	int dataIdx = 3;
	double vmax = 1.0;
	double amax = 3.0;
	// double vmax = 3.0;
	// double amax = 9.0;
	cout << "data index: " << dataIdx << " max vel: " << vmax << " max acc: " << amax << endl; 

	// read control points
	std::fstream controlPointFile;
	std::string controlPointFileName = "../data/control_points_data/control_point_divide" + std::to_string(dataIdx) + ".txt";
	controlPointFile.open(controlPointFileName, std::ios::in);

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
	std::string timeDivideFilename = "../data/divide_time_data/divide_time_" + std::to_string(dataIdx) + ".txt";
	timeDivideFile.open(timeDivideFilename, std::ios::in);

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
	std::string nearestObstacleFilename = "../data/nearest_obstacle_data/nearest_obstacles" + std::to_string(dataIdx) + ".txt";
	nearestObstacleFile.open(nearestObstacleFilename, std::ios::in);

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
			// cout << "ob: " << ob.transpose() << endl;
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
		// cout << "original acc: " << acc.norm() << endl;
		t += dt;
	}

	// for (int i=0; i<int(posData.size()); ++i){
	// 	cout << "pos: " << posData[i].transpose() << " , vel: " << velData[i].transpose() << ", acc: "  << accData[i].transpose() << endl;
	// }

	// velocity limits

	cout << "Total trajectory time is: " << t - dt << endl;

	
	timeOptimizer::timeOptimizer opt;
	ros::Time startTime = ros::Time::now();
	opt.setLimits(vmax, amax);
	opt.loadTrajectory(posData, velData, accData, dt);
	opt.loadTimeInterval(timeInterval);
	opt.loadObstacles(nearestObstacles);
	cout << "nearest obstacle size: " << nearestObstacles.size() << endl;
	bool success = opt.optimize();
	ros::Time endTime = ros::Time::now();
	cout << "total time: " << (endTime - startTime).toSec() << endl;
	cout << "success: " << success << endl;

	double alpha, beta;
	cout << "Real Time at 3.0 is: " << opt.remapTime(3.0, alpha, beta) << " alpha, beta: " << alpha << " " << beta << endl;
	cout << "Real Time at 30.0 is: " << opt.remapTime(30.0, alpha, beta) << " alpha, beta: " << alpha << " " << beta << endl;


	// obtain new trajectory
	t = 0.0;
	for (t=0.0; t<=opt.getDuration(); t+=dt){
		double alpha, beta;
		double optimizedTrajT = opt.remapTime(t, alpha, beta);
		double trajT = optimizedTrajT * linearReparamFactor;

		Eigen::Vector3d pos = traj.at(trajT);
		Eigen::Vector3d velReparam = traj.getDerivative().at(trajT) * linearReparamFactor;
		Eigen::Vector3d accReparam = traj.getDerivative().getDerivative().at(trajT) * pow(linearReparamFactor, 2);
		Eigen::Vector3d vel = velReparam * sqrt(beta);
		Eigen::Vector3d acc = velReparam * alpha + accReparam * beta;

		// cout << "beta: " << beta << endl;
		// cout << "alpha: " << alpha << endl;
		// cout << trajT << endl;
		// cout << "origin vel: " << velReparam.norm() << " acc: " << accReparam.norm() << endl; 
		cout << "Time: " << t << " pos: " << pos.transpose() << " vel: " << vel.transpose() << " acc: " << acc.transpose() << endl;
		// cout << "vel norm: " << vel.norm() << " acc norm: " << acc.norm() << endl;


	}
	return 0;
}