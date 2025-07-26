/*
Purpose: Concurrent Coverage Planner
Last updated: Dynamic TH_req
Last updated on: 12 Dec 2022
To do: ServiceClient array
Author: Ratijit Mitra
*/


#include "con_plan_exec_pkg/concpp_main.h"


using namespace ros;


void ConCPP_MAIN::initializeConCPP(ros::NodeHandle *nh)
{
	this->nh = nh;

	nh->getParam("ws_x", ws_size_x);
	nh->getParam("ws_y", ws_size_y);
	nh->getParam("rc", rob_count);
	TH_req = rob_count;
	cout << "Workspace size = " << ws_size_x << " x " << ws_size_y << endl;

	#ifdef TURTLEBOT
		cout << "#Robots = " << rob_count << " (TB)" << endl;
	#else
		cout << "#Robots = " << rob_count << " (QC)" << endl;
	#endif

	float_mat ws_init(ws_size_x, float_vec(ws_size_y, -1));		// -1 = Unexplored
	ws = ws_init;

	bool_mat assigned_goals_init(ws_size_x, bool_vec(ws_size_y, false));		// False = Unassigned
	assigned_goals = assigned_goals_init;

	obs_count = unassigned_goal_count = cov_count = 0;
	req_count = req_count_old = 0;

	bool_vec S_req_init(rob_count, false);		// False = Non-requester
	S_req = S_req_old = S_req_init;

	robot_status rob_state_init;
	rob_state_init.x = -1;
	rob_state_init.y = -1;;
	rob_state_init.theta = 0;

	robot_status_vec_t rob_states_init(rob_count, rob_state_init);
	rob_states = rob_states_old = rob_states_init;

	loc_mat paths_init(rob_count, loc_vec());
    paths = paths_init;

	start_from_intrvl = 1;

	uint_vec stops_at_clk_init(rob_count, 0);
	stops_at_clk = stops_at_clk_init;

	flag_running = false;
	call_id = 0;
	lv_ss = nh->advertiseService("share_workspace", &ConCPP_MAIN::getLocalView, this);
	// cout << "\nService Server = share_workspace started.";

	tot_comp_time = 0;
	last_req_exp_at_clk = 0;

	#ifdef DEBUG_RVIZ
		rviz_obj = new cRVIZ();
		rviz_obj -> init_rviz(ws_size_x, ws_size_y, ws, nh);
	#endif

	op_dir_path = ros::package::getPath("con_plan_exec_pkg") + OUTPUT_DIR;

	std::string rpc_file_path = op_dir_path + RESULT_PER_CALL_FILE_NAME;		// Empty resultPerCall.txt
	ofstream rpc_file;
	rpc_file.open(rpc_file_path.c_str());
	rpc_file.close();

	std::string cti_file_path = op_dir_path + COMP_TIME_INFO_FILE_NAME;		// Empty comp_time_info.txt
	ofstream cti_file;
	cti_file.open(cti_file_path.c_str());
	cti_file.close();

	std::string capl_file_path = op_dir_path + COL_AVRT_PATH_LENGTHS_FILE;			// Empty capl.txt
	ofstream capl_file;
	capl_file.open(capl_file_path.c_str());
	capl_file.close();

	initializeClock();
	boost::thread* write_clk_thread = new boost::thread(boost::bind(&ConCPP_MAIN::writeClock, this));
	write_clk_thread->detach();
}


bool ConCPP_MAIN::getLocalView(con_plan_exec_pkg::ShareLocalInformation::Request &req, con_plan_exec_pkg::ShareLocalInformation::Response &res)
{
	boost::thread* thread_1 = new boost::thread(boost::bind(&ConCPP_MAIN::handleRequest, this, req));
	thread_1->detach();

	return true;
}


void ConCPP_MAIN::handleRequest(con_plan_exec_pkg::ShareLocalInformation::Request req)
{
	#pragma omp critical
	{
		req_count++;

		if(!call_id && (req_count == rob_count))
		{
			mis_begun_at = ros::Time::now();
			clk_timer = nh->createTimer(ros::Duration(MOT_PREM_EXEC_TIME), &ConCPP_MAIN::incrementClock, this);
		}
		
		int rob_id = int(req.robot_id);
		S_req[rob_id] = true;
		rob_states[rob_id].x = int(req.x);
		rob_states[rob_id].y = int(req.y);
		rob_states[rob_id].theta = double(req.theta);

		updateGlobalView(req);
		// printGlobalView();

		// printf("\nR_%d (%d, %d, %1.0lf) LV_%d. #Req = %d", rob_id, rob_states[rob_id].x, rob_states[rob_id].y, rob_states[rob_id].theta, req.horizon, req_count);
		// cout << "\n#Req = " << req_count;
	}

	checkCoveragePlanningCriteria();
}


void ConCPP_MAIN::updateGlobalView(con_plan_exec_pkg::ShareLocalInformation::Request req)
{
	size_t lv_short_size = req.lv_short.size();

	for(size_t lv_index = 0; lv_index < lv_short_size; lv_index++)
	{
		con_plan_exec_pkg::CellInfo cell_info_tmp = req.lv_short[lv_index];
		float lv = float(cell_info_tmp.cell_type);		// Local view
		float gv_old = ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y];		// Old global view

		if(lv == 0)		// Obstacle
		{
			if(gv_old == -1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 0;
				obs_count++;
			}
		}
		else if(lv == 0.5)		// Goal
		{
			if(gv_old == -1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 0.5;
				unassigned_goal_count++;
			}
		}
		else		// Covered
		{
			if(gv_old != 1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 1;
				cov_count++;

				if(gv_old == 0)
					obs_count--;
				else if((gv_old == 0.5) && !assigned_goals[cell_info_tmp.cell_x][cell_info_tmp.cell_y])
					unassigned_goal_count--;
			}
		}
	}

	// printGlobalView();
}


void ConCPP_MAIN::printGlobalView()
{
	// cout << "\nGlobalView(" << call_id << ") ...\n";
	std::string gv_file_path = op_dir_path + "gv_" + std::to_string(call_id) + TXT_EXT;
	ofstream gv_file;
	gv_file.open(gv_file_path.c_str());

	for(uint row_id = 0; row_id < ws_size_x; row_id++)
	{
		for(uint col_id = 0; col_id < ws_size_y; col_id++)
		{
			// cout << ws[row_id][col_id] << " ";
			gv_file << ws[row_id][col_id] << ", ";
		}

		// cout << endl;
		gv_file << endl;
	}

	gv_file.close();
}


void ConCPP_MAIN::checkCoveragePlanningCriteria()
{
	#ifdef DEBUG_RVIZ
		rviz_obj -> update_rviz(ws);
	#endif
	
	#pragma omp critical
	{
		if(flag_running)
		{
			// cout << "\nRunning";
		}
		else
		{
			if(unassigned_goal_count)
			{
				if((!call_id && (req_count == rob_count)) || (call_id && (req_count >= TH_req)))
				{
					bool flag_robs_changed = false;

					if(req_count_old != req_count)
						flag_robs_changed = true;
					else
						for(uint rob_id = 0; rob_id < rob_count; rob_id++)
							if((!S_req_old[rob_id] && S_req[rob_id]) || (S_req_old[rob_id] && !S_req[rob_id]))
							{
								flag_robs_changed = true;
								break;
							}
							else if(S_req_old[rob_id] && S_req[rob_id])
							{
								robot_status rob_state_old = rob_states_old[rob_id];
								robot_status rob_state = rob_states[rob_id];

								if((rob_state_old.x != rob_state.x) || (rob_state_old.y != rob_state.y) || (rob_state_old.theta != rob_state.theta))
								{
									flag_robs_changed = true;
									break;
								}
							}

					if(flag_robs_changed)
					{
						flag_running = true;
						// printGlobalView();

						float_mat ws_old = ws;
						bool_mat assigned_goals_old = assigned_goals;
						uint obs_count_old = obs_count;
						uint unassigned_goal_count_old = unassigned_goal_count;
						uint cov_count_old = cov_count;

						S_req_old = S_req;
						req_count_old = req_count;
						req_count = 0;		// Reset

						rob_states_old = rob_states;

						robot_status rob_state;
						struct loc rob_init_state;		// Initial state of a Robot
						loc_vec req_rob_states;

						// cout << "\nProblem: #UG = " << unassigned_goal_count_old << " #Req = " << req_count_old;
						// cout << "\n\nRequesters: ";

						for(uint rob_id = 0; rob_id < rob_count; rob_id++)
							if(S_req_old[rob_id])
							{
								// cout << "R_" << rob_id << " ";
								S_req[rob_id] = false;		// Reset
								rob_state = rob_states_old[rob_id];

								rob_init_state.x = rob_state.x;
								rob_init_state.y = rob_state.y;
								rob_init_state.theta = (int)rob_state.theta;

								req_rob_states.push_back(rob_init_state);
							}
							// else
								// cout << "0 ";

						boost::thread* thread_2 = new boost::thread(boost::bind(&ConCPP_MAIN::startCoveragePlanning, this, ws_old, assigned_goals_old, obs_count_old, unassigned_goal_count_old, cov_count_old, req_rob_states));
						thread_2->detach();
					}
					else
					{
						// cout << "\nUnchanged";
					}
				}
				else
				{
					// cout << "\n#Req!";
				}
			}
			else if(req_count == rob_count)
			{
				// cout << "\nSTOP";
				stopCoveragePlanning();
			}
			else
			{
				// cout << "\n#UG!";
			}
		}
	}
}


void ConCPP_MAIN::startCoveragePlanning(float_mat ws_old, bool_mat assigned_goals_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, loc_vec req_rob_states)
{
	bool_mat ws_graph(ws_size_x, bool_vec(ws_size_y, true));		// true = Traversable (Covered / Goal), false = not Traversable (Obstacle / Unexplored)
	loc_vec unassigned_goal_locs;									// Locations of unassigned goals
	struct loc goal_loc;

	for(uint row_id = 0; row_id < ws_size_x; row_id++)
		for(uint col_id = 0; col_id < ws_size_y; col_id++)
		{
			float gv_old = ws_old[row_id][col_id];

			if((gv_old == -1) || (gv_old == 0))		// -1 = Unexplored, 0 = Obstacle
				ws_graph[row_id][col_id] = false;
			else if((gv_old == 0.5) && !assigned_goals_old[row_id][col_id])		// 0.5 = Goal
			{
				goal_loc.x = row_id;
				goal_loc.y = col_id;
				unassigned_goal_locs.push_back(goal_loc);
			}
		}

	//================================================== Cost Optimal Paths: START
	int_vec opt_goal_vec;
	uint act_count = 0;		// Number of active robots

	Time comp_begun_at = ros::Time::now();
	uint comp_begun_at_clk;

	#pragma omp critical
	comp_begun_at_clk = clock;

	GAP gap_obj;
	paths = gap_obj.get_cost_optimal_paths(ws_size_x, ws_size_y, ws_graph, req_count_old, req_rob_states, unassigned_goal_count_old, unassigned_goal_locs, call_id, S_req_old, paths, opt_goal_vec, act_count);		// Cost optimal paths of robots
	
	Time comp_ended_at = ros::Time::now();
	Duration comp_time = comp_ended_at - comp_begun_at;		// Computation time

	uint comp_ended_at_clk;

	#pragma omp critical
	comp_ended_at_clk = clock;
	//================================================== Cost Optimal Paths: END
	uint fsbl_comp_intrvl;			// Feasible paths computed at interval
	uint look_ahead_intrvl = 1;		// Number of intervals to be looked ahead
	bool flag_send_paths = false;

	while(!flag_send_paths)
	{
		//================================================== Collision Free Paths: START
		Time cfp_begun_at = ros::Time::now();
		uint cfp_begun_at_clk;

		#pragma omp critical
		cfp_begun_at_clk = clock;

		paths = gap_obj.get_col_free_paths(ws_size_x, ws_size_y, req_count_old, call_id, S_req_old, opt_goal_vec, paths, act_count, look_ahead_intrvl, start_from_intrvl, fsbl_comp_intrvl);		// Paths of Robots

		Time cfp_ended_at = ros::Time::now();
		Duration cfp_time = cfp_ended_at - cfp_begun_at;		// Computation time 2

		uint cfp_ended_at_clk;
		//================================================== Collision Free Paths: END
		uint cur_intrvl;		// Current Interval ID
		uint look_ahead_intrvl_bkp = look_ahead_intrvl;

		uint act_rob_count = 0;		// Number of active robots
		uint max_pl = 0;			// Maximum path length
		uint TH_req_old = TH_req;

		#pragma omp critical
		{
			cfp_ended_at_clk = clock;
			cur_intrvl = cfp_ended_at_clk / INTRVL_LEN_FAC;

			if(cur_intrvl < start_from_intrvl)		// Successful
			{
				flag_send_paths = true;
				TH_req = 0;		// Set dynamically

				loc_vec path;		// Path of a Robot
				uint path_len;
				struct loc rob_goal_state;
				uint send_lv_at_clk;

				for(uint rob_id = 0; rob_id < rob_count; rob_id++)
					if(S_req_old[rob_id])
					{
						path = paths[rob_id];
						path_len = path.size() - 1;

			    		if(path_len)		// Active
			    		{
			    			act_rob_count++;
		    				rob_goal_state = path[path_len];
			    			assigned_goals[rob_goal_state.x][rob_goal_state.y] = true;

			    			if(ws[rob_goal_state.x][rob_goal_state.y] != 1)		// Not covered yet
								unassigned_goal_count--;

							if(max_pl < path_len)
								max_pl = path_len;

							send_lv_at_clk = start_from_intrvl * INTRVL_LEN_FAC + path_len;
							stops_at_clk[rob_id] = send_lv_at_clk - 1;

							if(last_req_exp_at_clk < send_lv_at_clk)
							{
								last_req_exp_at_clk = send_lv_at_clk;
								last_req_exp_from_rob = rob_id;
							}

				    		boost::thread* thread_3 = new boost::thread(boost::bind(&ConCPP_MAIN::callRobot, this, call_id, start_from_intrvl, path_len, path, rob_id));		// Share paths simultaneously
							thread_3->detach();
			    		}
			    		else
		    			{
			    			S_req[rob_id] = true;
			    			req_count++;
			    			TH_req++;
			    		}
			    	}
			    	else
			    		if(stops_at_clk[rob_id] < cfp_ended_at_clk)
			    			TH_req++;

			    if(!TH_req)
			    {
			    	bool init = true;
			    	uint stops_at_clk_min;

			    	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			    	{
			    		uint stops_at_clk_tmp = stops_at_clk[rob_id];

			    		if(stops_at_clk_tmp >= cfp_ended_at_clk)
			    			if(init)
			    			{
			    				stops_at_clk_min = stops_at_clk_tmp;
			    				init = false;
			    			}
			    			else if(stops_at_clk_min > stops_at_clk_tmp)
			    				stops_at_clk_min = stops_at_clk_tmp;
			    	}

			    	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			    		if(stops_at_clk[rob_id] == stops_at_clk_min)
			    			TH_req++;
			    }
			}
			else
			{
				TH_req = req_count_old;

				Time cfp_exp_end_at = cfp_ended_at + cfp_time + Duration(0.001);		// Bias
				Duration cfp_exp_time = cfp_exp_end_at - mis_begun_at;
				uint cfp_exp_end_at_clk = cfp_exp_time.toSec() / MOT_PREM_EXEC_TIME;
				uint cfp_exp_end_at_intrvl = cfp_exp_end_at_clk / INTRVL_LEN_FAC;
				uint intrvl_dif = cfp_exp_end_at_intrvl - cur_intrvl;
				look_ahead_intrvl = 1 + intrvl_dif;		// Reactive
			}
		}

		printCallInformation(req_count_old, TH_req_old, obs_count_old, unassigned_goal_count_old, cov_count_old, comp_time.toSec(), comp_begun_at, comp_ended_at, comp_begun_at_clk, comp_ended_at_clk, cfp_time.toSec(), cfp_begun_at, cfp_ended_at, cfp_begun_at_clk, cfp_ended_at_clk, fsbl_comp_intrvl, look_ahead_intrvl_bkp, cur_intrvl, act_rob_count, max_pl, flag_send_paths);
		call_id++;
		tot_comp_time += comp_time.toSec() + cfp_time.toSec();
		comp_time = Duration(0);
	}

	// cout << "\n[R_" << last_req_exp_from_rob << " " << last_req_exp_at_clk << "]" << flush;

	#pragma omp critical
	flag_running = false;

	boost::thread* thread_5 = new boost::thread(boost::bind(&ConCPP_MAIN::checkCoveragePlanningCriteria, this));
	thread_5->detach();
}


void ConCPP_MAIN::callRobot(uint call_id_old, uint start_from_intrvl_old, uint path_len_old, loc_vec path_old, uint rob_id)
{
	con_plan_exec_pkg::PlanForHorizon pfh_srv;
	pfh_srv.request.hor_id = call_id;
	pfh_srv.request.start_from_intrvl = start_from_intrvl;

	con_plan_exec_pkg::PlanInstance pi_msg;
	struct loc rob_state;

	for(uint state_id = 0; state_id <= path_len_old; state_id++)
	{
		rob_state = path_old[state_id];
		pi_msg.x = rob_state.x;
		pi_msg.y = rob_state.y;
		pi_msg.theta = rob_state.theta;

		pfh_srv.request.plans.push_back(pi_msg);
	}

	std::string path_srv_name = "/robot_" + to_string(rob_id);
	path_srv_name += path_srv_name;
	path_srv_name += "/share_plan";

	ros::ServiceClient path_sc = nh->serviceClient<con_plan_exec_pkg::PlanForHorizon>(path_srv_name, true);
	path_sc.waitForExistence();
	
	if(path_sc.call(pfh_srv))
	{
		// if(pfh_srv.response.rcvd_intrvl >= pfh_srv.request.start_from_intrvl)
		// {
		// 	cout << "\nR_" << rob_id << " P_" << pfh_srv.request.hor_id << " delayed! CI (" << pfh_srv.response.rcvd_intrvl << ") >= SFI (" << pfh_srv.request.start_from_intrvl << ")\n" << flush;
		// 	ros::shutdown();
		// }
	}
	else
	{
		cout << endl << path_srv_name << "!" << flush;
		ros::shutdown();
	}
}


void ConCPP_MAIN::printCallInformation(uint req_count_old, uint TH_req_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, double comp_time, Time comp_begun_at, Time comp_ended_at, uint comp_begun_at_clk, uint comp_ended_at_clk, double cfp_time, Time cfp_begun_at, Time cfp_ended_at, uint cfp_begun_at_clk, uint cfp_ended_at_clk, uint fsbl_comp_intrvl, uint look_ahead_intrvl_bkp, uint cur_intrvl, uint act_rob_count, int max_pl, bool flag_send_paths)
{
	fstream rpc_file;
	std::string rpc_file_path = op_dir_path + RESULT_PER_CALL_FILE_NAME;
	rpc_file.open(rpc_file_path.c_str(), fstream::app);

	std::string cti_file_path = op_dir_path + COMP_TIME_INFO_FILE_NAME;
	fstream cti_file;
	cti_file.open(cti_file_path.c_str(), fstream::app);

	if(call_id)
	{
		rpc_file << endl;
		cti_file << endl;
	}

	rpc_file << "Call = " << call_id << ", #Req = " << req_count_old << " # " << TH_req_old << ", #O = " << obs_count_old << ", #UG = " << unassigned_goal_count_old << ", #C = " << cov_count_old << ", CT = " << fixed << (comp_time + cfp_time) << ", FCI = " << fsbl_comp_intrvl << ", LAI = " << look_ahead_intrvl_bkp << ", SFI = " << start_from_intrvl << ", CI = " << cur_intrvl << ", #Act = " << act_rob_count << ", Max PL = " << max_pl << ", " << flag_send_paths;
	cti_file << "Call = " << call_id << ", COP = " << fixed << comp_time << " # " << fixed << comp_begun_at << " : " << fixed << comp_ended_at << " # " << comp_begun_at_clk << " : " << comp_ended_at_clk << ", CFP = " << fixed << cfp_time << " # " << fixed << cfp_begun_at << " : " << fixed << cfp_ended_at << " # " << cfp_begun_at_clk << " : " << cfp_ended_at_clk;

	rpc_file.close();
	cti_file.close();

	cout << "\nCall = " << call_id << ", #Req = " << req_count_old << " # " << TH_req_old << ", #UG = " << unassigned_goal_count_old << ", #C = " << cov_count_old << ", CT = " << fixed << (comp_time + cfp_time) << ", " << fsbl_comp_intrvl << " + " << look_ahead_intrvl_bkp << " = " << start_from_intrvl << " > " << cur_intrvl << ", #Act = " << act_rob_count << ", Max PL = " << max_pl << ", " << flag_send_paths << endl;
	// cout << "\nC_" << call_id << " " << req_count_old << "->" << unassigned_goal_count_old << " " << act_rob_count << " " << cov_count_old << flush;
	// cout << "\n====================================================================================================";
}


void ConCPP_MAIN::stopCoveragePlanning()
{
	clk_timer.stop();
	Time mis_ended_at = ros::Time::now();
	Duration mis_time = mis_ended_at - mis_begun_at;		// Mission time
	stopRobots();

	ofstream r_file;
	std::string r_file_path = op_dir_path + RESULT_FILE;
	r_file.open(r_file_path.c_str());
	r_file << "Workspace Size = " << ws_size_x << " x " << ws_size_y << ", #Cov = " << cov_count << ", #Robots = " << rob_count << ", #Calls = " << call_id << ", Total Computation Time = " << fixed << tot_comp_time << ", Mission Time = " << fixed << mis_time.toSec() << " # " << fixed << mis_begun_at << " : " << fixed << mis_ended_at;
	
	#ifdef TURTLEBOT
		r_file << ", TurtleBot";
	#else
		r_file << ", QuadCopter";
	#endif

	r_file.close();

	cout << "\n\n******************** Coverage Completed ********************";
	cout << "\n#Calls = " << call_id << ", #Cov = " << cov_count << ", TCT = " << tot_comp_time << ", MT = " << mis_time.toSec() << endl;
	ros::shutdown();
}


void ConCPP_MAIN::writeClkFile(uint clk)
{
	ofstream clk_file;
	std::string clk_file_path = op_dir_path + CLOCK_FILE;
	clk_file.open(clk_file_path.c_str());
	clk_file << clk;
	clk_file.close();
}


void ConCPP_MAIN::initializeClock()
{
	clock = 0;
	writeClkFile(clock);
}


void ConCPP_MAIN::writeClock()
{
	ros::Publisher write_clk_pub = nh->advertise<con_plan_exec_pkg::DiscreteClock>("discrete_clock", 1000);
	con_plan_exec_pkg::DiscreteClock dc_msg;
	ros::Rate loop_rate(LOOP_RATE);

	while(ros::ok())
	{
		#pragma omp critical
		{
			dc_msg.clk_val = clock;
			// cout << "\nclock = " << clock;
		}

		write_clk_pub.publish(dc_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}


void ConCPP_MAIN::incrementClock(const ros::TimerEvent &te)
{
	#pragma omp critical
	{
		clock++;
		writeClkFile(clock);
	}
}


void ConCPP_MAIN::stopRobots()
{
	con_plan_exec_pkg::StopRobot sr_srv;
	sr_srv.request.stop_robot = true;

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		std::string sr_srv_name = "/stop_robot_" + to_string(rob_id);
		ros::ServiceClient sr_sc = nh->serviceClient<con_plan_exec_pkg::StopRobot>(sr_srv_name);

		if(!sr_sc.call(sr_srv))
			cout << endl << sr_srv_name << "!";
	}
}


int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "concpp_node");
	ros::NodeHandle nh("~");
	
	ConCPP_MAIN *concpp_main_obj = new ConCPP_MAIN();
	concpp_main_obj -> initializeConCPP(&nh);

	ros::spin();
	return 0;
}