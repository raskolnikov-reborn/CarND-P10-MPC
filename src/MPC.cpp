#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Setting values for simulation upfront
size_t N = 25;
double dt = 0.1;

// Penalty to be tuned for state change impact on cost
Eigen::Vector3d state_penalty(10.0, 0.5, 0.25);

// Penalty to be tuned for actuator motion impact on cost
Eigen:: Vector2d actuator_penalty(600000.0,33.0);

// Penalty to be tuned for sequential changes impact on cost
Eigen:: Vector2d transition_penalty(0.01,0.0001);


// Reference Target values for cte,epsi,ref_val
Eigen::Vector3d reference_vals(0,0,80.0);


size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {

		fg[0] = 0;

		// Cost Related to state
		Eigen::Vector3d state(cte_start,epsi_start,v_start);
		for (int i = 0; i < N; i++) {
			for (int j = 0; j <state.size(); j++ )
				fg[0] += state_penalty[j]*CppAD::pow(vars[state[j] + i] - reference_vals[j], 2);
		}

		// Cost Related to Actuator Motion
		Eigen::Vector2d actuators(delta_start, a_start);
		for (int i = 0; i < N - 1; i++) {
			for (int j = 0; j < actuators.size(); j++)
				fg[0] += actuator_penalty[j]*CppAD::pow(vars[actuators[j] + i], 2);
		}

		// Cost related to sequential changes
		for (int i = 0; i < N - 2; i++) {
			for (int j = 0; j < actuators.size(); j++)
				fg[0] += transition_penalty[j]*CppAD::pow(vars[actuators[j] + i + 1] - vars[actuators[j] + i], 2);
		}


		/// Using Code from the lesson as is

		// Initial constraints
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (int i = 0; i < N - 1; i++)
		{

			// The state at time t+1 .
			size_t offset_index = i + 1;
			AD<double> x1 = vars[x_start + offset_index];
			AD<double> y1 = vars[y_start + offset_index];
			AD<double> psi1 = vars[psi_start + offset_index];
			AD<double> v1 = vars[v_start + offset_index];
			AD<double> cte1 = vars[cte_start + offset_index];
			AD<double> epsi1 = vars[epsi_start + offset_index];

			// The state at time t.
			offset_index = i;
			AD<double> x0 = vars[x_start + offset_index];
			AD<double> y0 = vars[y_start + offset_index];
			AD<double> psi0 = vars[psi_start + offset_index];
			AD<double> v0 = vars[v_start + offset_index];
			AD<double> cte0 = vars[cte_start + offset_index];
			AD<double> epsi0 = vars[epsi_start + offset_index];

			// Only consider the actuation at time t.

			AD<double> delta0 = vars[delta_start + offset_index];
			AD<double> a0 = vars[a_start + offset_index];

			// Evaluate polynomial (x0 till time t)
			// useful in calculating [ f(x0) - y ]
			AD<double> f0 = 0.0;
			for (int i = 0; i < coeffs.size(); i++) {
				f0 += coeffs[i] * CppAD::pow(x0, i);
			}

			// Evaluate polynomial (x0 till time t-1)
			// useful in calculating [ f(psi0) - psides0 ]
			AD<double> psides0 = 0.0;
			for (int i = 1; i < coeffs.size(); i++) {
				psides0 += i*coeffs[i] * CppAD::pow(x0, i-1);
			}

			// Calculate tan inverse
			psides0 = CppAD::atan(psides0);

			// Set fg at appropriate index
			offset_index = i + 2;
			fg[x_start + offset_index] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[y_start + offset_index] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[psi_start + offset_index] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[v_start + offset_index] = v1 - (v0 + a0 * dt);
			fg[cte_start + offset_index] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[epsi_start + offset_index] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
		}

	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// TODO: Set the number of model variables (includes both states and inputs).

	// Assign State elements to semantically meaningful variable names
	// to avoid confusion while coding
	const double x = state[0];
	const double y = state[1];
	const double psi = state[2];
	const double v = state[3];
	const double cte = state[4];
	const double epsi = state[5];



	// Using weights_actuation (same size as actuators but available here)
	size_t n_vars = N*state.size() + (N-1)*actuator_penalty.size();


	// No. of constraints should be timesteps*state size
	size_t n_constraints = N*state.size();


	//Set all independent variables to zero
	Dvector vars(n_vars);
	for (int i = 0; i < n_vars; i++)
	{
		vars[i] = 0;
	}

	// Set initial values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	// Declaring lower and upperbound vectors
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);


	// Set upper and lower bounds for variables
	double d2r25 = 25.0*M_PI/180;
	for (i = 0; i < n_vars; i++)
	{
		if(i < delta_start)
		{
			vars_lowerbound[i] = -999.0f;
			vars_upperbound[i] = 999.0f;
		}
		else if( i < a_start)
		{
			// can at most be 25 degrees converted to radians
			vars_lowerbound[i] = -d2r25;
			vars_upperbound[i] = d2r25;
		}
		else
		{
			vars_lowerbound[i] = -1.0f;
			vars_upperbound[i] = 1.0f;
		}

	}


	// upper and lower bounds for constraints
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);


	// set all bounds to zero
	for (i = 0; i < n_constraints; i++)
	{
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	// Set lower bounds
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	// Set upper bounds
	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// Using options as is from lesson
	// NOTE: You don't have to worry about these options

	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";



	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// ipopt_solver
	CppAD::ipopt::solve<Dvector, FG_eval>(
			options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
			constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	// Commented out to reduce print load
	//	auto cost = solution.obj_value;
	//	std::cout << "Cost " << cost << std::endl;

	mpc_x_pts.clear();
	mpc_y_pts.clear();

	mpc_x_pts.resize(N);
	mpc_y_pts.resize(N);

	// Maintain current list of x and y outputs
	for (i = 0; i < N; i++)
	{
		mpc_x_pts[i] = solution.x[x_start + i];
		mpc_y_pts[i] = solution.x[y_start + i];
	}

	// return output as vector
	auto output = {solution.x[delta_start], solution.x[a_start] };

	return output;
}
