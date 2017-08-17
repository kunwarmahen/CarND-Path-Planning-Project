#include "Helper.h"

Helper::Helper() {
}

Helper::~Helper() {
}

void Helper::generate_trajectory(Vehicle& my_vehicle, vector<Vehicle>  other_vehicles, double prev_last_pos) {
	bool too_close = false;
	for (int i = 0; i < other_vehicles.size(); i++) {
		if (my_vehicle.get_car_lane() == other_vehicles.at(i).get_car_lane()) {
			vector<double> check_car_s = other_vehicles.at(i).state_at(prev_last_pos);
			if ((check_car_s.at(0) > my_vehicle.car_s) && ((check_car_s.at(0) - my_vehicle.car_s) < 30)) {
				too_close = true;
				if (my_vehicle.get_car_lane() > 0) {
					my_vehicle.set_car_lane(0);
					break;
				}
			}
		}
	}

	if (too_close) {
		my_vehicle.car_vel -= .224;
	}
	else if (my_vehicle.car_vel < 49.5) {
		my_vehicle.car_vel += .224;
	}
}


double Helper::logistic(double x) {
	return (2.0 / (1 + exp(-x)) - 1.0);
}

double  Helper::nearest_approach_to_any_vehicle(Vehicle &my_veh, vector<Vehicle> other_vehicles, double time) {
	double closest = 999999;
	for (int i = 0; i < other_vehicles.size(); i++) {
		if (my_veh.get_car_lane() == other_vehicles.at(i).get_car_lane() && other_vehicles.at(i).car_s > my_veh.car_s) {
			double d = nearest_approach(my_veh, other_vehicles.at(i), time);
			if (d > 0 && d < closest) {
				closest = d;
			}
		} 
	}
	return closest;
}

double Helper::nearest_approach(Vehicle &my_veh, Vehicle &other_vehicle, double time) {
	vector<double> other_state = other_vehicle.state_at(time);
	vector<double> my_state = my_veh.state_at(time);
	double fs_dist = other_state.at(0) - my_state.at(0);
	double s_dist = other_vehicle.car_s - my_veh.car_s;
	//cout << fs_dist << " " << s_dist << " " << my_veh.car_s << endl;
	if (fs_dist > 0) {
		if (fs_dist < s_dist)
			return fs_dist;
		else
			return s_dist;
	}
	else
		return s_dist;
}



double Helper::collision_cost(Vehicle &my_veh, vector<Vehicle> other_vehicle, double prev_last_pos) {

	double VEHICLE_RADIUS = 15;
	double nearest = nearest_approach_to_any_vehicle(my_veh, other_vehicle, prev_last_pos);

	if (nearest < 2 * VEHICLE_RADIUS)
		return 2.0;
	else
		return 0.0;
}

void Helper::generate_trajectory_1(Vehicle &my_vehicle, vector<Vehicle> other_vehicles, double prev_last_pos) {

	vector<Vehicle> trajs;

	Vehicle left_lane_vehicle;
	left_lane_vehicle.car_s = my_vehicle.car_s;
	left_lane_vehicle.car_vel = my_vehicle.car_vel;
	left_lane_vehicle.set_car_lane(0);
	trajs.push_back(left_lane_vehicle);

	Vehicle mid_lane_vehicle;
	mid_lane_vehicle.car_s = my_vehicle.car_s;
	mid_lane_vehicle.car_vel = my_vehicle.car_vel;
	mid_lane_vehicle.set_car_lane(1);
	trajs.push_back(mid_lane_vehicle);

	Vehicle right_lane_vehicle;
	right_lane_vehicle.car_s = my_vehicle.car_s;
	right_lane_vehicle.car_vel = my_vehicle.car_vel;
	right_lane_vehicle.set_car_lane(2);
	trajs.push_back(right_lane_vehicle);

	vector<double> traj_costs;
	for (int i = 0; i < trajs.size(); i++) {
		double cost = calculate_cost(trajs.at(i), my_vehicle, other_vehicles, prev_last_pos);
		//cout << i << " " << cost << endl;
		traj_costs.push_back(cost);
		cout << "Lane " << i << " ";
		cout << cost << endl;
	}

	// choose least-cost trajectory
	double min_cost = traj_costs[0];
	int min_cost_i = 0;
	for (int i = 0; i < traj_costs.size(); i++) {
		if (traj_costs[i] < min_cost) {
			min_cost = traj_costs[i];
			min_cost_i = i;
		}
	}

	cout << "Min cost for lane " << min_cost_i << endl;
	cout << "------------------" << endl;

	my_vehicle.set_car_lane(min_cost_i);

	/*if (too_close) {
		my_vehicle.car_vel -= .224;
	}
	else */if (my_vehicle.car_vel < 49.5) {
		my_vehicle.car_vel += .224;
	}
}

double Helper::calculate_cost(Vehicle &my_veh, Vehicle &orig_veh, vector<Vehicle> other_vehicles, double prev_last_pos) {
	
	double ex_sp_lim_cost = exceeds_speed_limit_cost(my_veh, other_vehicles, prev_last_pos);
	double ex_acc_lim_cost = exceeds_accel_cost(my_veh, other_vehicles, prev_last_pos);
	double ex_jerk_lim_cost = exceeds_jerk_cost(my_veh, other_vehicles, prev_last_pos);
	double col_cost = collision_cost(my_veh, other_vehicles, prev_last_pos);
	double lane_cost = lane_change_cost(my_veh, orig_veh);

	double infeasible_costs = ex_sp_lim_cost + ex_acc_lim_cost + ex_jerk_lim_cost + col_cost + lane_cost;

	return infeasible_costs;

}


double Helper::lane_change_cost(Vehicle &my_veh, Vehicle &orig_veh) {

	double cost = 0.0;
	if (orig_veh.get_car_lane() != my_veh.get_car_lane()) {
		cost += 0.35;
	}

	if (my_veh.get_car_lane() != 1) {
		cost +=  0.50;
	}

	return cost;
}


double Helper::exceeds_speed_limit_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos) {
	//for (int i = 0; i < _horizon; i++) {
	//	if (traj.first.eval_d(i) + traj.second.eval_d(i) > _hard_max_vel_per_timestep)
	//		return 1.0;
	//}
	return 0.0;
}

double Helper::exceeds_accel_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos) {
	//for (int i = 0; i < _horizon; i++) {
	//	if (traj.first.eval_double_d(i) + traj.second.eval_double_d(i) > _hard_max_acc_per_timestep)
	//		return 1.0;
	//}
	return 0.0;
}

double Helper::exceeds_jerk_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos) {
	//for (int i = 0; i < _horizon; i++) {
	//	if (traj.first.eval_triple_d(i) + traj.second.eval_triple_d(i) > _hard_max_jerk_per_timestep)
	//		return 1.0;
	//}
	return 0.0;
}

//double Helper::collision_cost(Vehicle my_veh, vector<Vehicle> const &vehicles) {
//	for (int t = 0; t < _horizon; t++) {
//		for (int i = 0; i < vehicles.size(); i++) {
//			double ego_s = traj.first.eval(t);
//			double ego_d = traj.second.eval(t);
//			vector<double> traffic_state = vehicles[i].state_at(t); // {s,d}
//
//			double dif_s = abs(traffic_state[0] - ego_s);
//			double dif_d = abs(traffic_state[1] - ego_d);
//
//			if ((dif_s <= _car_col_length) && (dif_d <= _car_col_width))
//				return 1.0;
//		}
//	}
//	return 0.0;
//}

