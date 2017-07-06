#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
    {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);

		// Commenting to reduce print load
		// cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry")
				{
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"] ;
					// Convert to meters per second from miles per hour
					v *=  0.44704;
					const double steering_angle = j[1]["steering_angle"];
					const double throttle = j[1]["throttle"];

					// Number of points
					const int N = ptsx.size();

					// Computing once to save on computation cost in for loop
					// Using negative steering (as per Project tips, Angle frame is reversed)
					const double cos_theta = cos(-psi);
					const double sin_theta = sin(-psi);

					// Using same parameters as in mpc.cpp
					const double dt = 0.1;
					const double Lf = 2.67;

					// List of points in local frame of reference
					Eigen::VectorXd x_local(N);
					Eigen::VectorXd y_local(N);

					// Create Points
					for(int i = 0; i < N; i++)
					{
						const double dx = ptsx[i] - px;
						const double dy = ptsy[i] - py;
						x_local[i] = dx * cos_theta - dy * sin_theta;
						y_local[i] = dy * cos_theta + dx * sin_theta;
					}

					// Get 3rd order polynomial
					auto coeffs = polyfit(x_local, y_local, 3);

					// set cte and epsi
					const double cte = coeffs[0];
					const double epsi = -atan(coeffs[1]);

					// update state using kinematic model to deal with latency
					// Everything is in vehicle frame
					const double delta_x = v * dt;
					const double delta_y = 0;
					const double delta_psi = - v * steering_angle * dt / Lf;
					const double v_new = v + throttle * dt;
					const double cte_new = cte + v * sin(epsi) * dt;
					const double  epsi_new = epsi + delta_psi;

					Eigen::VectorXd state(6);
					state << delta_x, delta_y, delta_psi, v_new, cte_new, epsi_new;
					vector<double> mpc_results = mpc.Solve(state, coeffs);

					// Calculate Steering and throttle
					// Ensure output is between - 1 and 1
					double steer_value = mpc_results[0]/ deg2rad(25);
					double throttle_value = mpc_results[1];


					json msgJson;

					// Set Steering and throttle values to send to Simulator
					msgJson["steering_angle"] = -steer_value;
					msgJson["throttle"] = throttle_value;

					//Display the MPC predicted trajectory
					vector<double> mpc_x_vals = mpc.mpc_x_pts;
					vector<double> mpc_y_vals = mpc.mpc_y_pts;


					// Send mpc trajectory
					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

					//Display the output reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// Display waypoint reference line
					for(int i = 0; i<ptsx.size();i++)
					{
						next_x_vals.push_back(x_local[i]);
						next_y_vals.push_back(y_local[i]);
					}

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";

					//Commenting print to reduce print load
					//std::cout << msg << std::endl;

					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.

					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.

					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING
					this_thread::sleep_for(chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
