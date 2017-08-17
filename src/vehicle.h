#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class Vehicle {
public:
	Vehicle();
	virtual ~Vehicle();
	double car_s;
	double car_d;
	double car_vel;
	int suggested_lane;
	double current_car_s;

	int get_car_lane();
	void set_car_lane(int lane);
	vector<double> state_at(double t);
	void reset_suggested_lane();

private:
	int lane;
};

#endif /* VEHICLE_H */

