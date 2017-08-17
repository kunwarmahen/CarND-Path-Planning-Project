#include "vehicle.h"

Vehicle::Vehicle() {
	car_s = 0.0;
	car_d = 0.0;
	car_vel = 0.0;

	current_car_s = 0.0;
	suggested_lane = 100;

	lane = 1;
}

Vehicle::~Vehicle() {
}

int Vehicle::get_car_lane() {
	
	if (this->car_d > 0 && this->car_d <= 4) {
		lane = 0;
	}
	else if (this->car_d > 4 && this->car_d <= 8) {
		lane = 1;
	}
	else {
		lane = 2;
	}

	return lane;
}

void Vehicle::set_car_lane(int lane) {
	this->lane = lane;
	this->car_d = 2 + 4 * lane;
}

vector<double> Vehicle::state_at(double time)  {
	return { this->car_s + ((double)time * 0.02 * this->car_vel),  this->car_d};
}

void Vehicle::reset_suggested_lane() {
	if (this->get_car_lane() == this->suggested_lane) {
		this->suggested_lane = 100;
	}
}