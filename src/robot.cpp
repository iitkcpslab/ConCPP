/*
Purpose: Concurrent Robot
Last updated: Follow path in sync with the global clock
Last updated on: 13 Dec 2022
To do: 
Author: Ratijit Mitra
*/


#include <ros/ros.h>
#include <ros/package.h>


#include <math.h>
#include <string>
#include <stdlib.h>
#include <vector> 
#include <iostream>
#include <fstream>
#include <string.h>


#include "con_plan_exec_pkg/basics.h"
#include "con_plan_exec_pkg/config.h"
#include "con_plan_exec_pkg/debug.h"

#include "con_plan_exec_pkg/PlanForHorizon.h"
#include "con_plan_exec_pkg/ShareLocalInformation.h"
#include "con_plan_exec_pkg/StopRobot.h"

#include "con_plan_exec_pkg/DiscreteClock.h"
#include "con_plan_exec_pkg/CellInfo.h"


#define WS_OBS_ROBS_FILE_NAME "ws_obs_robs.txt"		// Workspace with Obstacles and Robots
#define DEBUG_ACC


using namespace ros;


class RobotClass
{	
	public:
		ros::NodeHandle *nh;

		int rob_id;			// Robot ID

		float loc_x;		// Current state
		float loc_y;
		float loc_theta;

		uint ws_size_x;			// Workspace size
		uint ws_size_y;
		float_mat ws_lidar;		// LiDAR values

		uint req_id;					// Request ID
		bool first_req_sent;
		ros::ServiceClient lv_sc;		// Localview service client

		float_mat lv;									// Localview
		map<pair<uint, uint>, float> cell_info_map;		// Localview short

		ros::ServiceServer path_ss;		// Path service server
		plan_vec_t path;				// Path
		uint path_len;
		uint path_len_rem;

		uint start_from_clk;		// Start following the path from the beginning of the clock
		uint send_lv_at_clk;		// Send localview at clk

		#ifdef DEBUG_ACC
		std::string acc_file_path;
		#endif

		uint prev_clk;			// Previous CLK
		uint cur_clk;			// Current CLK

		uint cur_intrvl;		// Current interval ID

		ros::ServiceServer stop_rob_ss;		// Stop robot service server


		void initializeRobot(ros::NodeHandle *nh)
		{
			this->nh = nh;
			nh->getParam("rid", rob_id);

			req_id = 0;
			first_req_sent = false;
			lv_sc = nh->serviceClient<con_plan_exec_pkg::ShareLocalInformation>("/concpp_node/share_workspace", false);

			cell_info_map.clear();
			path_len_rem = 0;

			start_from_clk = prev_clk = send_lv_at_clk = 0;

			#ifdef DEBUG_ACC
			acc_file_path = ros::package::getPath("con_plan_exec_pkg") + "/output/acc_" + std::to_string(rob_id) + ".txt";
			ofstream acc_file;
			acc_file.open(acc_file_path.c_str());
			acc_file.close();
			#endif
		}


		void populateLiDARValues()
		{
			std::string ws_obs_robs_file_path = ros::package::getPath("con_plan_exec_pkg") + INPUT_DIR_NAME + WS_OBS_ROBS_FILE_NAME;
			std::ifstream ws_obs_robs_file;
		    ws_obs_robs_file.open(ws_obs_robs_file_path.c_str()); 
		  	std::string line;
		  	float lidar_val;
		  	uint row_id = 0, col_id;

		    while(ws_obs_robs_file)
		    {
		        getline(ws_obs_robs_file, line);
		        char *token = strtok(&line[0], ",");
		        col_id = 0;
		        float_vec ws_lidar_row;

		        while(token)
		        {
		        	lidar_val = atof(token);		// 0 = Obstacle, 0.5 = Free, >= 1 = Robot ID

		        	if(lidar_val >= 1)		// Robot IDs
		        	{
		        		if(int(lidar_val) == (rob_id + 1))			// Robot IDs start from 0
		        		{
		        			loc_x = row_id;
							loc_y = col_id;
							loc_theta = 1;		// 0 = E, 1 = N, 2 = W, 3 = S
		        		}
			        	
			        	ws_lidar_row.push_back(0.5);
		        	}
		        	else
			        	ws_lidar_row.push_back(lidar_val);
		        	
		        	token = strtok(NULL, ",");
		        	col_id++;
		        	
	        		if(ws_size_y < col_id)
	        			ws_size_y = col_id;
		        }

		        row_id++;
		        ws_lidar.push_back(ws_lidar_row);
		    }

		    ws_size_x = row_id - 1;
		    ws_obs_robs_file.close();

		    #ifdef TURTLEBOT
				cout << "R_" << rob_id << " (" << loc_x << ", " << loc_y << ", " << loc_theta << ") in " << ws_size_x << " x " << ws_size_y << endl << endl;
			#else
				cout << "R_" << rob_id << " (" << loc_x << ", " << loc_y << ") in " << ws_size_x << " x " << ws_size_y << endl << endl;
			#endif

		    float_mat lv_init(ws_size_x, float_vec(ws_size_y, -1));		// Initialize: -1 = Unexplored
		    lv = lv_init;
		}
		
		
		void readLiDAR(int nbr_x, int nbr_y)
		{
			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y))		// Valid
				if(lv[nbr_x][nbr_y] == -1)		// Only update unexplored cells
				{
					float lidar_val = ws_lidar[nbr_x][nbr_y];
					lv[nbr_x][nbr_y] = lidar_val;

					pair<uint, uint> cell_loc_key(nbr_x, nbr_y);

					if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
					{
						pair<pair<uint, uint>, float> cell_info(cell_loc_key, lidar_val);
						cell_info_map.insert(cell_info);
					}
				}
		}
		
		
		void updateLocalView()
		{
			int cur_x = int(loc_x);
	        int cur_y = int(loc_y);

	        readLiDAR(cur_x + 1, cur_y);		// East cell
	        readLiDAR(cur_x, cur_y + 1);		// North cell
	        readLiDAR(cur_x - 1, cur_y);		// West cell
	        readLiDAR(cur_x, cur_y - 1);		// South cell

			lv[cur_x][cur_y] = 1;			// Robot's current cell is covered

			pair<uint, uint> cell_loc_key(cur_x, cur_y);

			if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
			{
				pair<pair<uint, uint>, float> cell_info(cell_loc_key, 1);
				cell_info_map.insert(cell_info);
			}
			else
				cell_info_map.at(cell_loc_key) = 1;

			// printLocalView();
		}
		
		
		void printLocalView()
		{
			// cout << "\nR_" << rob_id << " Localview...\n";

			for(uint row_id = 0; row_id < ws_size_x; row_id++)
			{
				for(uint col_id; col_id < ws_size_y; col_id++)
					cout << lv[row_id][col_id] << " ";

				// cout << endl;
			}
		}


		void startServiceServers()
		{
			std::string path_srv_name = "robot_" + std::to_string(rob_id) + "/share_plan";
			path_ss = nh->advertiseService(path_srv_name, &RobotClass::receivePath, this);
			// cout << "\nPath Service Server = " << path_srv_name;

			std::string stop_rob_srv_name = "/stop_robot_" + to_string(rob_id);
			stop_rob_ss = nh->advertiseService(stop_rob_srv_name, &RobotClass::stopRobot, this);
			// cout << "\nStop robot Service Server = " << stop_rob_srv_name;
		}


		void readClock(const con_plan_exec_pkg::DiscreteClock& msg)
		{
			#pragma omp critical
			{
				cur_clk = msg.clk_val;

				if(!first_req_sent)
				{
					first_req_sent = true;
					sendLocalView();
				}

				if(prev_clk < cur_clk)		// Incremented
				{
					prev_clk = cur_clk;

					if(cur_clk >= start_from_clk)
					{
						if(cur_clk < send_lv_at_clk)
						{
							#ifdef TURTLEBOT
								goToState(path[cur_clk - start_from_clk + 1], cur_clk);
					        #else			            
								goToCardinalState(path[cur_clk - start_from_clk + 1], cur_clk);
					        #endif

							path_len_rem--;

							#ifdef DEBUG_ACC
							fstream acc_file;
							acc_file.open(acc_file_path.c_str(), ios::app);
							acc_file << path_len_rem << endl;
							acc_file.close();
							#endif
						}
						else if(cur_clk == send_lv_at_clk)
							sendLocalView();
						else if(path_len)		// Delayed
						{
							cout << "R_" << rob_id << " LV_" << req_id << " delayed! CLK(" << cur_clk << ") > SEND_LV_CLK(" << send_lv_at_clk << ")" << endl;
							system("rosnode kill concpp_node");
							ros::shutdown();
						}
					}
				}

				cur_intrvl = cur_clk / INTRVL_LEN_FAC;
			}
		}
		
		
		void sendLocalView()
		{
			path.clear();
			path_len = 0;

			if(path_len_rem)
			{
				cout << "R_" << rob_id << " RPL = " << path_len_rem << endl;
				system("rosnode kill concpp_node");
				ros::shutdown();
			}

			#ifdef DEBUG_ACC
			fstream acc_file;
			acc_file.open(acc_file_path.c_str(), ios::app);
			acc_file << "SENT\n";
			acc_file.close();
			#endif
			
			con_plan_exec_pkg::ShareLocalInformation lv_srv;
			lv_srv.request.robot_id = rob_id;
			lv_srv.request.horizon = req_id;
			lv_srv.request.x = loc_x;
	    	lv_srv.request.y = loc_y;
			lv_srv.request.theta = loc_theta;
			
			// for(uint row_id = 0; row_id < ws_size_x; row_id++)
			// 	for(uint col_id = 0; col_id < ws_size_y; col_id++)
			// 		lv_srv.request.workspace.push_back(lv[row_id][col_id]);

			con_plan_exec_pkg::CellInfo cell_info_tmp;
			map<pair<uint, uint>, float>::iterator cell_info_map_it;
			
			for(cell_info_map_it = cell_info_map.begin(); cell_info_map_it != cell_info_map.end(); ++cell_info_map_it)
			{
				// cout << endl << (cell_info_map_it->first).first << "," << (cell_info_map_it->first).second << ": " << cell_info_map_it->second;
				cell_info_tmp.cell_x = (cell_info_map_it->first).first;
				cell_info_tmp.cell_y = (cell_info_map_it->first).second;
				cell_info_tmp.cell_type = cell_info_map_it->second;
				lv_srv.request.lv_short.push_back(cell_info_tmp);
			}
			
			lv_sc.waitForExistence();

			if(lv_sc.call(lv_srv))
				req_id++;
			else
			{
				cout << "R_" << lv_srv.request.robot_id << " (" << lv_srv.request.x << ", " << lv_srv.request.y << ", " << lv_srv.request.theta << ") sendLocalView(" << lv_srv.request.horizon << ")!" << endl;
				uint lv_short_size = lv_srv.request.lv_short.size();
				con_plan_exec_pkg::CellInfo cell_info_msg;

				for(uint index = 0; index < lv_short_size; index++)
				{
					cell_info_msg = lv_srv.request.lv_short[index];
					cout << endl << cell_info_msg.cell_x << ", " << cell_info_msg.cell_y << " = " << cell_info_msg.cell_type;
				}

				system("rosnode kill concpp_node");
				ros::shutdown();
			}
		}


		bool receivePath(con_plan_exec_pkg::PlanForHorizon::Request &req, con_plan_exec_pkg::PlanForHorizon::Response &res)
		{
			#pragma omp critical
			{
				uint sfi_tmp = req.start_from_intrvl;

				if(cur_intrvl < sfi_tmp)		// Must receive the path in advance
				{
					path_len_rem = path_len = req.plans.size() - 1;
					motionPlan tmp_plan;
					con_plan_exec_pkg::PlanInstance pi_msg;
					 
					for(uint state_id = 0; state_id <= path_len; state_id++)
					{
						pi_msg = req.plans[state_id];
						tmp_plan.location_x = pi_msg.x;
						tmp_plan.location_y = pi_msg.y;
						tmp_plan.theta = pi_msg.theta;
						path.push_back(tmp_plan);
					}

					start_from_clk = sfi_tmp * INTRVL_LEN_FAC;
					send_lv_at_clk = start_from_clk + path_len;
					// cout << "R_" << rob_id << " accepted Response_" << req.hor_id << "[" << start_from_clk << "-" << start_from_clk + path_len - 1 << "]";

					#ifdef DEBUG_ACC
					fstream acc_file;
					acc_file.open(acc_file_path.c_str(), ios::app);
					acc_file << cur_intrvl << " Res_" << req.hor_id << ": " << start_from_clk << " " << path_len << endl;
					acc_file.close();
					#endif

					cell_info_map.clear();
				}
				else
				{
					cout << "R_" << rob_id << " Response_" << req.hor_id << " delayed! CI (" << cur_intrvl << ") >= SFI (" << sfi_tmp << ")" << endl;
					system("rosnode kill concpp_node");
					ros::shutdown();
				}

				res.rcvd_intrvl = cur_intrvl;
		    }

	  		return true;
		}


		void goToState(motionPlan state, uint t)
		{
			int loc_theta_int = int(loc_theta);

			float loc_x_next = state.location_x;
			float loc_y_next = state.location_y;
			float loc_theta_next = state.theta;
			int loc_theta_next_int = int(loc_theta_next);

			if(loc_theta == loc_theta_next)
			{
				if((loc_x == loc_x_next) && (loc_y == loc_y_next))
				{
					cout << "R_" << rob_id << " W (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") @ CLK = " << t + 1 << endl;
				}
				else if(((loc_x + 1) == loc_x_next) || ((loc_x - 1) == loc_x_next) || ((loc_y + 1) == loc_y_next) || ((loc_y - 1) == loc_y_next))
				{
					cout << "R_" << rob_id << " MF (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") @ CLK = " << t + 1 << endl;
				}
				else
				{
					cout << "R_" << rob_id << " Invalid move! (" << int(loc_x) << ", " << int(loc_y) << ", " << loc_theta_int << ") to (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") = " << t + 1 << endl;
					system("rosnode kill concpp_node");
					ros::shutdown();
				}
			}
			else
			{
				if(((loc_theta_int + 1) % 4) == loc_theta_next_int)
				{
					cout << "R_" << rob_id << " TL (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") @ CLK = " << t + 1 << endl;
				}
				else if(((loc_theta_next_int + 1) % 4) == loc_theta_int)
				{
					cout << "R_" << rob_id << " TR (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") @ CLK = " << t + 1 << endl;
				}
				else
				{
					cout << "R_" << rob_id << " Invalid move! (" << int(loc_x) << ", " << int(loc_y) << ", " << loc_theta_int << ") to (" << int(loc_x_next) << ", " << int(loc_y_next) << ", " << loc_theta_next_int << ") @ CLK = " << t + 1 << endl;
					system("rosnode kill concpp_node");
					ros::shutdown();
				}
			}

			loc_x = loc_x_next;
			loc_y = loc_y_next;
			loc_theta = loc_theta_next;
			loc_theta_int = int(loc_theta);
			
			updateLocalView();
		}


		void goToCardinalState(motionPlan state, uint t)
		{
			float loc_x_next = state.location_x;
			float loc_y_next = state.location_y;
			float loc_theta_next = state.theta;

			if((loc_x == loc_x_next) && (loc_y == loc_y_next))
			{
				cout << "R_" << rob_id << " W (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
			}
			else
				if(loc_x == loc_x_next)
					if((loc_y + 1) == loc_y_next)
					{
						cout << "R_" << rob_id << " MN (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
					}
					else if((loc_y - 1) == loc_y_next)
					{
						cout << "R_" << rob_id << " MS (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
					}
					else
					{
						cout << "R_" << rob_id << " Invalid move! (" << int(loc_x) << ", " << int(loc_y) << ") to (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
						system("rosnode kill concpp_node");
						ros::shutdown();
					}
				else if(loc_y == loc_y_next)
					if((loc_x + 1) == loc_x_next)
					{
						cout << "R_" << rob_id << " ME (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
					}
					else if((loc_x - 1) == loc_x_next)
					{
						cout << "R_" << rob_id << " MW (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
					}
					else
					{
						cout << "R_" << rob_id << " Invalid move! (" << int(loc_x) << ", " << int(loc_y) << ") to (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
						system("rosnode kill concpp_node");
						ros::shutdown();
					}
				else
				{
					cout << "R_" << rob_id << " Invalid move! (" << int(loc_x) << ", " << int(loc_y) << ") to (" << int(loc_x_next) << ", " << int(loc_y_next) << ") @ CLK = " << t + 1 << endl;
					system("rosnode kill concpp_node");
					ros::shutdown();
				}

			loc_x = loc_x_next;
			loc_y = loc_y_next;
			loc_theta = loc_theta_next;

			updateLocalView();
		}


		bool stopRobot(con_plan_exec_pkg::StopRobot::Request &req, con_plan_exec_pkg::StopRobot::Response &res)
		{
			if(req.stop_robot)
			{
				cout << "\nR_" << rob_id << " Coverage Completed (" << req_id << ")" << endl;
				ros::shutdown();
			}

			return true;
		}
};


int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "robot_node");		// Rename using __name:=robot_i in the rosrun command, where i (>= 0) is the ID of Robot_i
	ros::NodeHandle nh("~");
	
	RobotClass *robot_obj = new RobotClass();
	robot_obj->initializeRobot(&nh);
	robot_obj->populateLiDARValues();
	robot_obj->updateLocalView();
	robot_obj->startServiceServers();
	ros::Subscriber clk_sub = nh.subscribe("/concpp_node/discrete_clock", 1000, &RobotClass::readClock, robot_obj);

	ros::spin();
	return 0;
}