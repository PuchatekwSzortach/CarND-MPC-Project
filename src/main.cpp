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


// Get coefficients of a straight line that's perpendicular to provided line and passes through point (x, y)
Eigen::VectorXd get_perpendicular_line_passing_given_point(
  Eigen::VectorXd polynomial_coefficients, double x, double y)
{
  // Find equation of line perpendicular to waypoints line that passes through point (x, y)
  double perpendicular_line_slope = -1.0 / polynomial_coefficients[1] ;
  double perpendicular_line_offset = y - (perpendicular_line_slope * x) ;

  Eigen::VectorXd perpendicular_line_coefficients(2) ;
  perpendicular_line_coefficients << perpendicular_line_offset, perpendicular_line_slope ;

  return perpendicular_line_coefficients ;
}


// Get cross track error
double get_cross_track_error(double x, double y, std::vector<double> ptsx, std::vector<double> ptsy)
{
  // We will approximate cross track error by fitting a line to first two waypoints and computing
  // shortest distance from vehicle to that line
  Eigen::VectorXd ptsx_eigen(2) ;
  Eigen::VectorXd ptsy_eigen(2) ;

  ptsx_eigen << ptsx[0], ptsx[1] ;
  ptsy_eigen << ptsy[0], ptsy[1] ;

  // Fit a line into first two waypoints
  Eigen::VectorXd polynomial_coefficients = polyfit(ptsx_eigen, ptsy_eigen, 1) ;

  Eigen::VectorXd perpendicular_line_coefficients =
    get_perpendicular_line_passing_given_point(polynomial_coefficients, x, y) ;

  // Find an intersection of waypoints line and perpendicular line
  double lines_intersection_x =
    (perpendicular_line_coefficients[0] - polynomial_coefficients[0]) /
    (polynomial_coefficients[1] - perpendicular_line_coefficients[1]) ;

  double lines_intersection_y =
    (perpendicular_line_coefficients[1] * lines_intersection_x) + perpendicular_line_coefficients[0] ;

  double x_difference = x - lines_intersection_x ;
  double y_difference = y - lines_intersection_y ;

  // CTE is then a distance between (x, y) and point of intersection of waypoints line and perpendicular line that
  // passes through (x, y)
  return std::sqrt((x_difference * x_difference) + (y_difference * y_difference)) ;
}

double get_normalized_angle(double angle)
{
  while(angle < -M_PI) { angle += 2 * M_PI ; }
  while(angle > M_PI) { angle -= 2 * M_PI ; }

  return angle ;
}


double get_steering_error(double psi, std::vector<double> ptsx, std::vector<double> ptsy)
{
  double waypoints_heading = std::atan2(ptsy[1] - ptsy[0], ptsx[1] - ptsx[0]) ;
  double heading_difference = psi - waypoints_heading ;

  return get_normalized_angle(heading_difference) ;
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

//          Eigen::VectorXd ptsy_eigen(ptsx.size());
//          Eigen::VectorXd ptsx_eigen(ptsx.size());

//          // Copy std::vector content to eigen vector
//          for (int index = 0; index < ptsx.size(); ++index) {
//            ptsx_eigen[index] = ptsx[index];
//            ptsy_eigen[index] = ptsy[index];
//          }

          Eigen::VectorXd ptsx_eigen(2) ;
          Eigen::VectorXd ptsy_eigen(2);

          // We will only use two points for now
          ptsx_eigen << ptsx[1], ptsx[3] ;
          ptsy_eigen << ptsy[1], ptsy[3] ;

          double cte = get_cross_track_error(px, py, ptsx, ptsy);
          double epsi = get_steering_error(psi, ptsx, ptsy);

          // Construct state vector
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          // Compute control commands
          vector<double> control_commands ;

          Eigen::VectorXd polynomial_coefficients = polyfit(ptsx_eigen, ptsy_eigen, 1);
          control_commands = mpc.Solve(state, polynomial_coefficients);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
//          double steer_value = 0 ;
//          double throttle_value = 0 ;

          double steer_value = control_commands[6] ;
          double throttle_value = control_commands[7] ;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(int index = 0 ; index < ptsx_eigen.size() ; ++index)
          {
            double x_global_coordinates = ptsx_eigen[index] ;

            double y_global_coordinates =
                polynomial_coefficients[0] + (polynomial_coefficients[1] * x_global_coordinates) ;

            double x_delta = x_global_coordinates - px ;
            double y_delta = y_global_coordinates - py ;

            double distance = std::sqrt((x_delta * x_delta) + (y_delta * y_delta)) ;

            // Angle between point and x coordinates
            double angle = atan2(y_delta, x_delta) - psi ;

            double x = distance * std::cos(angle) ;
            double y = distance * std::sin(angle) ;

            mpc_x_vals.push_back(x) ;
            mpc_y_vals.push_back(y) ;
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals ;
          vector<double> next_y_vals ;

          // Calculate waypoints in car coordinate system
          for(int index = 0 ; index < ptsx.size() ; ++index)
          {
            // Distance from point to vehicle in global coordinate system
            double x_delta = ptsx[index] - px ;
            double y_delta = ptsy[index] - py ;
            double distance = std::sqrt((x_delta * x_delta) + (y_delta * y_delta)) ;

            // Angle between point and x coordinates
            double angle = atan2(y_delta, x_delta) - psi ;

            double x = distance * std::cos(angle) ;
            double y = distance * std::sin(angle) ;

            next_x_vals.push_back(x) ;
            next_y_vals.push_back(y) ;
          }


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.

          int delay = 0 ;
          this_thread::sleep_for(chrono::milliseconds(delay));
          std::cout << "DELAY SET TO 0! REMEMBER TO CHANGE THIS BACK LATER!" << std::endl ;

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
