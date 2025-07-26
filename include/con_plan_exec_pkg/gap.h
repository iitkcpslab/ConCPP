/*
Purpose: Asynchronous Coverage Planner
Last updated: 
Last updated on: 12 Dec 2022
To do: 
Author: Ratijit Mitra
*/


#include <stdio.h>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>


#include <ros/ros.h>
#include <ros/package.h>


#include "con_plan_exec_pkg/ma.h"
#include "con_plan_exec_pkg/config.h"
#include "con_plan_exec_pkg/debug.h"


#define COL_AVRT_PATH_LENGTHS_FILE "capl.txt"
#define CLOCK_FILE "clock.txt"


using namespace std;


class GAP
{
	public:
		loc_mat get_cost_optimal_paths(int ws_size_x, int ws_size_y, bool_mat ws_graph, uint num_of_robs, loc_vec robs_states, uint goal_count, loc_vec goals_locs, uint call_id, bool_vec S_r, loc_mat paths_old, int_vec &opt_goal_vec, uint &active_count);
		loc_mat get_col_free_paths(int ws_size_x, int ws_size_y, uint num_of_robs, uint call_id, bool_vec S_r, int_vec opt_goal_vec, loc_mat opt_paths, uint active_count, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl);
		
		void showQueue(queue<struct loc> BFS_QUEUE);
		void showMatrix(int_mat v_2d);
		
		int_mat bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc cur_rob_state);
		int_mat bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc cur_rob_state);
		int_mat compute_optimal_costs(bool_mat ws_graph, loc_vec robs_states, loc_vec goals_locs);
		
		loc_vec optimal_path_bfs(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc rob_loc, struct loc goal_loc);
		loc_vec optimal_path_bfs_longitudinal(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc rob_loc, struct loc goal_loc);
		loc_mat compute_optimal_paths(bool_mat ws_graph, loc_vec robs_states, loc_vec goals_locs, int_vec opt_goal_vec, bool_vec S_r, loc_mat paths_old, uint &active_count);
		
		bool is_type1_crossover_path_pair(int rob_1, int rob_2, struct loc rob1_start_loc, loc_vec path_2);
		bool is_type2_crossover_path_pair(int rob_1, int rob_2, loc_vec path_1, loc_vec path_2);

		bool is_nested_path_pair(int rob_1, int rob_2, loc_vec path_1, loc_vec path_2);
		void compute_residual_paths(loc_mat &opt_paths, bool_vec S_r, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl);
		loc_vec adjust_path(struct loc init_state, loc_vec opt_path);
		bool test_path(int i, loc_vec path, vec_int goal_vec_new, loc_mat feasible_paths, bool_vec S_r);
		loc_mat get_feasible_paths(int_vec &opt_goal_vec, loc_mat opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y, bool_vec S_r, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl);

		bool_mat compute_partial_orders(uint num_of_robs, loc_mat paths, bool_vec S_r);

		int_vec compute_total_order(int num_of_robs, bool_mat adj, bool_mat &adj_residue);

		void break_dependency_cycles(int_vec &opt_goal_vec, bool_mat po_residue, bool_vec S_r);
		void adjust_dependent_paths(int_vec opt_goal_vec, loc_mat &paths, bool_vec S_r, uint &active_count);

		int_vec compute_start_time_offsets(int_vec total_order, loc_mat paths, bool_vec S_r, int_vec &opt_goal_vec, bool &flag_inactivated_robot);

		loc_mat compute_optimal_trajectories(int num_of_robs, int_vec time_offsets, loc_mat paths, bool_vec S_r, uint horizon);

		void monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, loc_mat trajectories);
};