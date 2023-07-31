/*
*	File: timeOptimizer.h
*	---------------
*   time optimizer implementation
*/

#include <time_optimizer/timeOptimizer.h>

namespace timeOptimizer{
	timeOptimizer::timeOptimizer(){}
	

	void timeOptimizer::setLimits(double vmax, double amax){
		this->vmax_ = vmax;
		this->amax_ = amax;
	}

	void timeOptimizer::loadTrajectory(const std::vector<Eigen::Vector3d>& posData, 
							const std::vector<Eigen::Vector3d>& velData,
							const std::vector<Eigen::Vector3d>& accData,
							double dt){
		this->posData_ = posData;
		this->velData_ = velData;
		this->accData_ = accData;
		this->dt_ = dt;
	}

	void timeOptimizer::loadTimeInterval(const std::vector<std::pair<double, double>>& timeInterval){
		this->timeInterval_ = timeInterval;
	}

	void timeOptimizer::divideData(std::vector<std::vector<Eigen::Vector3d>>& posDataList, 
						std::vector<std::vector<Eigen::Vector3d>>& velDataList,
						std::vector<std::vector<Eigen::Vector3d>>& accDataList,
						std::vector<bool>& obstacleInfoList){
		cout << "start divide data based on the time interval." << endl;

		if (this->timeInterval_.size() == 0){
			posDataList.push_back(this->posData_);
			velDataList.push_back(this->velData_);
			accDataList.push_back(this->accData_);
			obstacleInfoList.push_back(false);
		}
		else{
			double t = 0;
			int intervalIdx = 0;
			bool prevInObstacleRange = false;
			bool inObstacleRange = false;
			std::vector<Eigen::Vector3d> posDataDivide, velDataDivide, accDataDivide;
			for (int i=0; i<int(this->posData_.size()); ++i){
				std::pair<double, double> interval = this->timeInterval_[intervalIdx];
				if (t >= interval.first and t <= interval.second){
					inObstacleRange = true;
				}
				else{
					inObstacleRange = false;
				}

				posDataDivide.push_back(this->posData_[i]);
				velDataDivide.push_back(this->velData_[i]);
				accDataDivide.push_back(this->accData_[i]);

				if (i == 0){
					if (inObstacleRange){
						obstacleInfoList.push_back(true);
					}
					else{
						obstacleInfoList.push_back(false);
					}
				}
				else{
					if (prevInObstacleRange and not inObstacleRange){
						posDataList.push_back(posDataDivide);
						velDataList.push_back(velDataDivide);
						accDataList.push_back(accDataDivide);
						obstacleInfoList.push_back(false);

						posDataDivide = std::vector<Eigen::Vector3d> {this->posData_[i]};
						velDataDivide = std::vector<Eigen::Vector3d> {this->velData_[i]};
						accDataDivide = std::vector<Eigen::Vector3d> {this->accData_[i]};
						if (intervalIdx != int(this->timeInterval_.size()-1)){
							++intervalIdx;
						}
					}
					else if (not prevInObstacleRange and inObstacleRange){
						posDataList.push_back(posDataDivide);
						velDataList.push_back(velDataDivide);
						accDataList.push_back(accDataDivide);
						obstacleInfoList.push_back(true);

						posDataDivide = std::vector<Eigen::Vector3d> {this->posData_[i]};
						velDataDivide = std::vector<Eigen::Vector3d> {this->velData_[i]};
						accDataDivide = std::vector<Eigen::Vector3d> {this->accData_[i]};						
					}
				}
				prevInObstacleRange = inObstacleRange;

				t += this->dt_;
			}

			posDataList.push_back(posDataDivide);
			velDataList.push_back(velDataDivide);
			accDataList.push_back(accDataDivide);
			obstacleInfoList.push_back(prevInObstacleRange);
		}

	}
	
	bool timeOptimizer::optimize(){
		std::vector<std::vector<Eigen::Vector3d>> posDataList, velDataList, accDataList;
		std::vector<bool> obstacleInfoList;
		this->divideData(posDataList, velDataList, accDataList, obstacleInfoList);

		// data check
		// double t = 0;
		// for (int i=0; i<int(posDataList.size()); ++i){
		// 	std::vector<Eigen::Vector3d> posDataDivide = posDataList[i];
		// 	std::vector<Eigen::Vector3d> velDataDivide = velDataList[i];
		// 	std::vector<Eigen::Vector3d> accDataDivide = accDataList[i];
		// 	cout << "In obstacle: " << obstacleInfoList[i] << endl;
		// 	for (int j=0; j<int(posDataDivide.size()); ++j){
		// 		Eigen::Vector3d pos = posDataDivide[j];
		// 		Eigen::Vector3d vel = velDataDivide[j];
		// 		Eigen::Vector3d acc = accDataDivide[j];
		// 		cout << "t: " << t << " pos: " << pos.transpose() << endl;
		// 		if (j != int(posDataDivide.size())-1){
		// 			t += this->dt_;	
		// 		}
		// 	}
		// }


		// variable order: alpha, beta, zeta, gamma, s
		int varNum = 0;
		int alphaNum = 0;
		int betaNum = 0;
		int zetaNum = 0;
		int gammaNum = 0;
		std::vector<int> alphaNumVec, betaNumVec, zetaNumVec, gammaNumVec;

		// linear constraints
		int linearConNum = 0;
		int alphaBetaConNum = 0;
		int velLimitsConNum = 0;
		int accLimitsConNum = 0;
		int velBoundaryConNum = 2;
		int accBoundaryConNum = 2;
		int velContinuityConNum = int(posDataList.size()) - 1;
		int accContinuityConNum = int(posDataList.size()) - 1;

		// conic constraints
		int coneNum = 0;
		int numafe = 0;
		int betaZetaConeNum = 0; // (0.5, beta_i, zeta_i) -> Q_r3
		int gammaZetaConeNum = 0; // (gamma_i, zeta_i + zeta_i+1, sqrt(2)) -> Q_r3
		int sAlphaConeNum = 1; // (0.5, s, alpha) -> Q_r(2+alphaNum)
		for (int i=0; i<int(posDataList.size()); ++i){
			int K = int(posDataList[i].size()) - 1;
			alphaNumVec.push_back(K); alphaNum += K;
			betaNumVec.push_back(K+1); betaNum += K+1;
			zetaNumVec.push_back(K+1); zetaNum += K+1;
			gammaNumVec.push_back(K); gammaNum += K;

			alphaBetaConNum += K;
			velLimitsConNum += K+1; 
			accLimitsConNum += K;

			betaZetaConeNum += K+1;
			gammaZetaConeNum += K;
		}
		varNum = alphaNum + betaNum + zetaNum + gammaNum + 1; // last slack variable for alpha
		linearConNum = alphaBetaConNum + velLimitsConNum + accLimitsConNum + velBoundaryConNum + velBoundaryConNum + accBoundaryConNum + velContinuityConNum + accContinuityConNum;
		coneNum = betaZetaConeNum + gammaZetaConeNum + sAlphaConeNum;
		numafe = 3 * betaZetaConeNum + 3 * gammaZetaConeNum + (2 + alphaNum);

		// create mosek environment
		MSKrescodee r;
		MSKenv_t env = NULL;
		r = MSK_makeenv(&env, NULL);
		
		// create task
		MSKtask_t task = NULL;
		if (r == MSK_RES_OK){
			r = MSK_maketask(env, 0, 0, &task);

			if (r == MSK_RES_OK){
				MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);

				// create variables
				if (r == MSK_RES_OK){
					r = MSK_appendvars(task, varNum); // all variable will be fixed at x = 0
				}

				// // assign names to each variable
				// {
				// 	int varNameCount = 0;
				// 	// alpha
				// 	for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
				// 		int K = int(posDataList[n].size()) - 1;
				// 		for (int i=0; i<K; ++i){
				// 			std::string name = "a_" + std::to_string(n) + "^" + std::to_string(i);
				// 			MSK_putvarname(task, varNameCount, name.c_str());
				// 			++varNameCount;
				// 		}
				// 	}

				// 	// beta
				// 	for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
				// 		int K = int(posDataList[n].size()) - 1;
				// 		for (int i=0; i<K+1; ++i){
				// 			std::string name = "b_" + std::to_string(n) + "^" + std::to_string(i);
				// 			MSK_putvarname(task, varNameCount, name.c_str());					
				// 			++varNameCount;
				// 		}
				// 	}

				// 	// zeta
				// 	for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
				// 		int K = int(posDataList[n].size()) - 1;
				// 		for (int i=0; i<K+1; ++i){
				// 			std::string name = "z_" + std::to_string(n) + "^" + std::to_string(i);
				// 			MSK_putvarname(task, varNameCount, name.c_str());					
				// 			++varNameCount;
				// 		}
				// 	}
					
				// 	// gamma
				// 	for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
				// 		int K = int(posDataList[n].size()) - 1;
				// 		for (int i=0; i<K; ++i){
				// 			std::string name = "y_" + std::to_string(n) + "^" + std::to_string(i);
				// 			MSK_putvarname(task, varNameCount, name.c_str());					
				// 			++varNameCount;
				// 		}
				// 	}

				// 	// s
				// 	std::string sName = "s";
				// 	MSK_putvarname(task, varNum-1, sName.c_str());
				// }

				// set linear objective sum(2 * dt * gamma) + lambda * s * dt
				if (r == MSK_RES_OK){ // for gamma
					for (int i=alphaNum+betaNum+zetaNum; i<alphaNum+betaNum+zetaNum+gammaNum and r==MSK_RES_OK; ++i){
						r = MSK_putcj(task, i, 2.0 * this->dt_);
					}
				}
				if (r == MSK_RES_OK){ // for slack variable s
					r = MSK_putcj(task, varNum-1, this->lambda_ * this->dt_);
				}
				if (r == MSK_RES_OK){
					r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
				}


				// set all variables to unconstrained
				if (r == MSK_RES_OK){
					r = MSK_putvarboundsliceconst(task, 0, varNum, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
				}

				// set beta > 0
				if (r == MSK_RES_OK){
					r = MSK_putvarboundsliceconst(task, alphaNum, alphaNum+betaNum, MSK_BK_LO, 0.0, +MSK_INFINITY);
				}

				// create linear constraints
				if (r == MSK_RES_OK){
					r = MSK_appendcons(task, linearConNum);
				}
				int currContraintNum = 0;
				// cout << "alpha beta" << endl;
				// 1. alpha beta
				int subi1[3] = {0, alphaNum, alphaNum+1};
				double vali1[3] = {1.0, 1.0/this->dt_, -1.0/this->dt_};
				for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					for (int i=0; i<K; ++i){
						r = MSK_putarow(task, currContraintNum, 3, subi1, vali1);
						if (r == MSK_RES_OK){
							r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, 0.0, 0.0);
						}
						subi1[0] += 1;
						subi1[1] += 1;
						subi1[2] += 1;						
						++currContraintNum;
					}
					subi1[1] += 1;
					subi1[2] += 1; // to move beta 1 more position because of changing pieces
				}


				// cout << "vel" << endl;
				// 2. velocity limits
				int subi2[1] = {alphaNum}; 
				for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					for (int i=0; i<K+1; ++i){	
						Eigen::Vector3d vel = velDataList[n][i];
						double velSum = vel.norm();
						double vali2[1] = {pow(velSum, 2)};
						r = MSK_putarow(task, currContraintNum, 1, subi2, vali2);
						if (r == MSK_RES_OK){
							r = MSK_putconbound(task, currContraintNum, MSK_BK_UP, -MSK_INFINITY, pow(this->vmax_, 2));
						}
						subi2[0] += 1;
						++currContraintNum;
					}
				}


				// cout << "acc" << endl;
				// 3. acceleration limits
				int subi3[2] = {0, alphaNum}; 
				for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					for (int i=0; i<K; ++i){	
						Eigen::Vector3d vel = velDataList[n][i];
						Eigen::Vector3d acc = accDataList[n][i];
						double velSum = vel.norm();
						double accSum = acc.norm();
						double vali3[2] = {velSum, accSum};
						r = MSK_putarow(task, currContraintNum, 2, subi3, vali3);
						if (r == MSK_RES_OK){
							r = MSK_putconbound(task, currContraintNum, MSK_BK_RA, -this->amax_, this->amax_);
						}
						subi3[0] += 1;
						subi3[1] += 1;
						++currContraintNum;
					}
					subi3[1] += 1;
				}

				// cout << "vel boundary" << endl;
				// 4. velocity boundary conditions
				// V0
				int subi4[1] = {alphaNum}; // for first beta
				double initVelX = velDataList[0][0](0);
				double vali4[1] = {pow(initVelX, 2)};
				if (r == MSK_RES_OK){
					r = MSK_putarow(task, currContraintNum, 1, subi4, vali4);
				}
				if (r == MSK_RES_OK){
					r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, pow(initVelX, 2), pow(initVelX, 2));
				}
				++currContraintNum;

				// Vf
				subi4[0] = alphaNum + betaNum - 1;
				double endVelX = velDataList.back().back()(0);
				vali4[0] = pow(endVelX, 2);
				if (r == MSK_RES_OK){
					r = MSK_putarow(task, currContraintNum, 1, subi4, vali4);
				}
				if (r == MSK_RES_OK){
					r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, pow(endVelX, 2), pow(endVelX, 2));
				}				
				++currContraintNum;

				// cout << "acc boundary" << endl;
				// 5. acceleration boundary conditions
				int subi5[2] = {0, alphaNum}; // for first alpha and beta
				double initAccX = accDataList[0][0](0);
				double vali5[2] = {initVelX, initAccX};
				if (r == MSK_RES_OK){
					r = MSK_putarow(task, currContraintNum, 2, subi5, vali5);
				}
				if (r == MSK_RES_OK){
					r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, initAccX, initAccX);
				}
				++currContraintNum;

				subi5[0] = alphaNum - 1;
				subi5[1] = alphaNum + betaNum - 1;
				double endAccX = accDataList.back().back()(0);
				vali5[0] = endVelX;
				vali5[1] = endAccX;
				if (r == MSK_RES_OK){
					r = MSK_putarow(task, currContraintNum, 2, subi5, vali5);
				}
				if (r == MSK_RES_OK){
					r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, endAccX, endAccX);
				}
				// ++currContraintNum;

				// int subi5[1] = {0};
				// double vali5[1] = {1.0};
				// if (r == MSK_RES_OK){
				// 	r = MSK_putarow(task, currContraintNum, 1, subi5, vali5);
				// }
				// if (r == MSK_RES_OK){
				// 	r == MSK_putconbound(task, currContraintNum, MSK_BK_FX, 0.0, 0.0);
				// }
				// ++currContraintNum;	

				// subi5[0] = {alphaNum-1};
				// if (r == MSK_RES_OK){
				// 	r = MSK_putarow(task, currContraintNum, 1, subi5, vali5);
				// }
				// if (r == MSK_RES_OK){
				// 	r == MSK_putconbound(task, currContraintNum, MSK_BK_FX, 0.0, 0.0);
				// }
				++currContraintNum;				

				// cout << "vel conitnuity" << endl;
				// 6. velocity continuity constraints
				int subi6[2] = {alphaNum, alphaNum};
				double vali6[2] = {1.0, -1.0};
				for (int n=0; n<int(posDataList.size())-1 and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					subi6[0] = subi6[1] + K;
					subi6[1] = subi6[0] + 1;
					r = MSK_putarow(task, currContraintNum, 2, subi6, vali6);
					if (r == MSK_RES_OK){
						r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, 0.0, 0.0);
					}
					++currContraintNum;
				}

				// cout << "acc conitnuity" << endl;
				// 7. acceleration conitnuity constraints
				int subi7[2] = {0, 0};
				double vali7[2] = {1.0, -1.0};
				for (int n=0; n<int(posDataList.size())-1 and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					subi7[0] = subi7[1] + K - 1;
					subi7[1] = subi7[0] + 1;
					r = MSK_putarow(task, currContraintNum, 2, subi7, vali7);
					if (r == MSK_RES_OK){
						r = MSK_putconbound(task, currContraintNum, MSK_BK_FX, 0.0, 0.0);
					}
					++currContraintNum;
				}

				// conic constraints	
				MSKint64t domidx[coneNum] = {0};
				int countDomain = 0;
				if (r == MSK_RES_OK){
					r = MSK_appendafes(task, numafe);
				}	

				// cout << "beta zeta cone" << endl;
				// beta zeta conic constraints (0.5, beta_i, zeta_i) -> Q_r(3)
				int currConeConstraintNum = 0;
				int betaIdx1 = alphaNum;
				int zetaIdx1 = alphaNum + betaNum;
				for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					for (int i=0; i<K+1; ++i){
						r = MSK_putafeg(task, currConeConstraintNum, 0.5);
						++currConeConstraintNum;
						if (r == MSK_RES_OK){
							r = MSK_putafefentry(task, currConeConstraintNum, betaIdx1, 1.0);
						}
						++currConeConstraintNum;
						if (r == MSK_RES_OK){
							r = MSK_putafefentry(task, currConeConstraintNum, zetaIdx1, 1.0);
						}
						++currConeConstraintNum;
						
						betaIdx1 += 1;
						zetaIdx1 += 1;

						// assign cone domain
						if (r == MSK_RES_OK){
							r = MSK_appendrquadraticconedomain(task, 3, domidx+countDomain);
						}
						++countDomain;
					}
				}

				// cout << "gamma zeta cone" << endl;
				// gamma zeta conic constraints (gamma_i, zeta_i + zeta_i+1, sqrt(2)) -> Q_r(3)
				int gammaIdx2 = alphaNum + betaNum + zetaNum;
				int zetaIdx2 = alphaNum + betaNum;
				for (int n=0; n<int(posDataList.size()) and r==MSK_RES_OK; ++n){
					int K = int(posDataList[n].size()) - 1;
					for (int i=0; i<K; ++i){
						r = MSK_putafefentry(task, currConeConstraintNum, gammaIdx2, 1.0);
						++currConeConstraintNum;
						if (r == MSK_RES_OK){
							r = MSK_putafefentry(task, currConeConstraintNum, zetaIdx2, 1.0);
							r = MSK_putafefentry(task, currConeConstraintNum, zetaIdx2+1, 1.0);
						} 
						++currConeConstraintNum;
						if (r == MSK_RES_OK){
							r = MSK_putafeg(task, currConeConstraintNum, sqrt(2));
						}
						++currConeConstraintNum;

						gammaIdx2 += 1;
						zetaIdx2 += 1;

						if (r == MSK_RES_OK){
							r = MSK_appendrquadraticconedomain(task, 3, domidx+countDomain);
							++countDomain;
						}
					}
					zetaIdx2 += 1;
				}

				// cout << "s alpha cone" << endl;
				// s and alpha conic constraint (s, 0.5, alpha) -> Q_r(2+alphaNum)
				if (r == MSK_RES_OK){ // s
					r = MSK_putafefentry(task, currConeConstraintNum, varNum-1, 1.0);
				}
				++currConeConstraintNum;
				
				if (r == MSK_RES_OK){ // 0.5
					r = MSK_putafeg(task, currConeConstraintNum, 0.5);
				}
				++currConeConstraintNum;
				
				int alphaIdx3 = 0;
				for (int i=0; i<alphaNum and r == MSK_RES_OK; ++i){ // all alpha
					r = MSK_putafefentry(task, currConeConstraintNum, alphaIdx3, 1.0);
					++currConeConstraintNum;
					++alphaIdx3;
				}

				if (r == MSK_RES_OK){
					r = MSK_appendrquadraticconedomain(task, 2+alphaNum, domidx+countDomain);
				}
				++countDomain;



				// lastly, append all cone in sequence
				if (r == MSK_RES_OK){
					r = MSK_appendaccsseq(task, coneNum, domidx, numafe, 0, NULL);
				}


				// cout << "check the number of constraints." << endl;
				// cout << "Expected number of linear constraints: " << linearConNum << " " << "Added constraint num: " << currContraintNum << endl;
				// cout << "Expected cone constraints: " << numafe << " " << "added cone constraint num: " << currConeConstraintNum << endl; 
				// MSK_writedata(task,"data.ptf");

				MSK_putintparam(task, MSK_IPAR_INFEAS_REPORT_AUTO, MSK_ON);
				if (r == MSK_RES_OK){
					MSKrescodee trmcode;
					r = MSK_optimizetrm(task, &trmcode);
					MSK_solutionsummary(task, MSK_STREAM_MSG);
					MSKsolstae solsta;
					if (r == MSK_RES_OK){
						r = MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					}

					switch (solsta){
						case MSK_SOL_STA_OPTIMAL:
						{
							double *xx = (double*) calloc(varNum, sizeof(double));
							if (xx){
								MSK_getxx(task, MSK_SOL_ITR, xx);
								cout << "[TimeOptimizer]: Optimal allocation obtained." << endl;

								this->extractSol(xx);
								free(xx);
								MSK_deletetask(&task);
								return true;
							}
						}
						case MSK_SOL_STA_DUAL_INFEAS_CER:
							cout << "[TimeOptimizer]: Dual infeasible." << endl;
			            case MSK_SOL_STA_PRIM_INFEAS_CER:
			            	cout << "[TimeOptimizer]: Primal or dual infeasibility certificate found." << endl;
			              	break;
			            case MSK_SOL_STA_UNKNOWN:
			            	cout << "[TimeOptimizer]: The status of the solution could not be determined. Termination code: " << trmcode << endl;
			              	break;
			            default:
			            	cout << "[TimeOptimizer]: Other solution status." << endl;
			              	break;
					}
				}
			}			
		}
		MSK_deletetask(&task);
		return false;		
	}

	void timeOptimizer::extractSol(double* sol){

	}

	double timeOptimizer::remapTime(double tau){
		return 0;
	}
}