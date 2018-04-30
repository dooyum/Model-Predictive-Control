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
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];

          // Factor in latency for delay in actuation.
          double latency = 0.1; // 100ms.
          double Lf = 2.67;
          v *= 0.447; // convert from mph to m/s.
          // px, py, psi and v 100ms into the future.
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          psi = psi + (v * delta * latency) / Lf;

          vector<double> vehicle_ptsx = {};
          vector<double> vehicle_ptsy = {};
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            // Get the difference in x and y values of waypoints to global position of vehicle.
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;
            double theta = 0 - psi; // Rotate perspective clockwise.

            // Transform to vehicle coordinates
            double vehicle_x = x_diff * cos(theta) - y_diff * sin(theta);
            double vehicle_y = y_diff * cos(theta) + x_diff * sin(theta);
            vehicle_ptsx.push_back(vehicle_x);
            vehicle_ptsy.push_back(vehicle_y);
          }

          Eigen::VectorXd ptsx_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vehicle_ptsx.data(), vehicle_ptsx.size());
          Eigen::VectorXd ptsy_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vehicle_ptsy.data(), vehicle_ptsy.size());

          // Gets coefficients that fit all x and y values to a 3rd degree polynomial.
          auto coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);
          // Psi is always 0 w.r.t. the vehicle coordniates.
          // x is 0 w.r.t the vehicle coordinates, so all derivate values of the polynomial with an x can be removed.
          // i.e. C2 * x + C3 * x^2 = 0
          // epsi and epsi after latency are equal w.r.t the vehicle coordinates.
          double epsi = -atan(coeffs[1]);
          // No need to subrtact vehicle observed y from derived y because, y should be 0 w.r.t. vehicle.
          double cte = polyeval(coeffs, 0);
          double latency_cte = cte + v * sin(epsi) * latency;
          double latency_x = v * latency;

          Eigen::VectorXd state(6);
          // The vehicle state w.r.t the vehicle is positioned at [x,y] = [0,0] at a 0 degree angle.
          state << latency_x, 0, 0, v, latency_cte, epsi;
          auto vars = mpc.Solve(state, coeffs);
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // Divide by deg2rad(25) to bring values between [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (unsigned int i = 2; i < vars.size(); i++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Number of future way points to show.
          double num_way_points = 15;
          double point_gap = 1.5;
          for (unsigned int x_point = 0; x_point < num_way_points; x_point++) {
            next_x_vals.push_back(x_point * point_gap);
            next_y_vals.push_back(polyeval(coeffs, x_point * point_gap));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car doesn't actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
