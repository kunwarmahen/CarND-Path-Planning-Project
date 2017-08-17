#include <vector>
#include <iostream>

using namespace std;

class Trajectory {
public:
	Trajectory();
	virtual ~Trajectory();
	vector<double> future_state;
	vector<double> s_coefficients;
	vector<double> d_coefficients;
};
