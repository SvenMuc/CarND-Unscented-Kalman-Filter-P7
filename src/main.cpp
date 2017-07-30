#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

/**
 * Prints the command line parameter usage.
 *
 * @param name Program name.
 */
static void show_usage(std::string name)
{
  cerr << "Usage: " << name << " <option(s)> SOURCES"
  << "Options:\n"
  << "\t-h,--help\t\tShow this help message\n"
  << "\t-o,--output OUTPUT\tSpecify the output file\n"
  << "\t-l,--lidar\tUse lidar data only\n"
  << "\t-r,--radar\tUse radar data only"
  << endl;
}

/**
 * Fusion mode
 *
 * - LIDAR_ONLY   Uses LIDAR data only.
 * - RADAR_ONLY   Uses RADAR data only.
 * - FUSION       Fuses LIDAR and RADAR data.
 */
enum FusionMode{
  LIDAR_ONLY,
  RADAR_ONLY,
  FUSION
};

/**
 * Main routine.
 *
 * @param argc Number of arguments.
 * @param argv Arguments list.
 * @return tbd.
 */
int main(int argc, char** argv)
{
  // check arguments
  if (argc > 4) {
    show_usage(argv[0]);
    return 1;
  }
  
  string output;
  ofstream output_file;
  FusionMode fusion_mode = FUSION;
  
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    
    if ((arg == "-h") || (arg == "--help")) {
      show_usage(argv[0]);
      return 0;
    } else if ((arg == "-o") || (arg == "--output")) {
      // setup output file
      if (i + 1 < argc) {
        output = argv[++i];
        output_file.open(output, ios_base::out);
        
        if (!output_file.is_open()) {
          cout << "Unable to create file " << output << "." << endl;
          return 1;
        } else {
          cout << "Created output file: " << output << endl;
        }
      } else {
        cerr << "--output option requires one argument." << endl;
        return 1;
      }
    } else if ((arg == "-l") || (arg == "--lidar")) {
      // setup LIDAR only mode
      if (fusion_mode == RADAR_ONLY) {
        cerr << "LIDAR only mode cannot be combined with RADAR only mode.";
        return 1;
      }
      
      fusion_mode = LIDAR_ONLY;
      cout << "Fusion Mode: LIDAR only" << endl;
      
    } else if ((arg == "-r") || (arg == "--radar")) {
      // setup radar only mode
      if (fusion_mode == LIDAR_ONLY) {
        cerr << "RADAR only mode cannot be combined with LIDAR only mode.";
        return 1;
      }
      
      fusion_mode = RADAR_ONLY;
      cout << "Fusion Mode: RADAR only" << endl;
    }
  }

  uWS::Hub h;
  
  // Create a Kalman Filter instance and configure sensor setup
  UKF ukf;
  
  switch (fusion_mode) {
    case LIDAR_ONLY:
      ukf.use_lidar_ = true;
      ukf.use_radar_ = false;
      break;

    case RADAR_ONLY:
      ukf.use_lidar_ = false;
      ukf.use_radar_ = true;
      break;

    default:
      ukf.use_lidar_ = true;
      ukf.use_radar_ = true;
      break;
  }
  
  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  
  h.onMessage([&ukf, &tools, &estimations, &ground_truth, &output_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      
      auto s = hasData(std::string(data));
      if (s != "") {
        
        auto j = json::parse(s);
        
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;
          
          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;
          float meas_px = 0.0;
          float meas_py = 0.0;
          
          if (sensor_type.compare("L") == 0) {
            // prepare LIDAR measurement data
            meas_package.sensor_type_ = MeasurementPackage::LASER;
          	meas_package.raw_measurements_ = VectorXd(2);
          	float px;
            float py;
          	iss >> px;
          	iss >> py;
            meas_px = px;
            meas_py = py;
          	meas_package.raw_measurements_ << px, py;
          	iss >> timestamp;
          	meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {
            // prepare RADAR measurement data
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
          	meas_package.raw_measurements_ = VectorXd(3);
          	float ro;
            float theta;
            float ro_dot;
          	iss >> ro;
          	iss >> theta;
          	iss >> ro_dot;
            meas_px = ro * cos(theta);
            meas_py = ro * sin(theta);
          	meas_package.raw_measurements_ << ro,theta, ro_dot;
          	iss >> timestamp;
          	meas_package.timestamp_ = timestamp;
          }
          
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
          ukf.ProcessMeasurement(meas_package);
          
          //Push the current estimated x,y positon from the Kalman filter's state vector
          
          VectorXd estimate(4);
          
          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);
          
          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;
          
          double NIS_radar = ukf.NIS_radar_;
          double NIS_lidar = ukf.NIS_lidar_;
          
          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
          
          estimations.push_back(estimate);
          
          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          
          // write UKF estimates and ground truth to output file:
          // File format: sensor est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy rmse_x rmse_x rmse_vx rmse_vy nis_radar nis_lidar
          if (output_file.is_open()) {
            if ((sensor_type.compare("L") == 0 && ukf.use_lidar_) ||
                (sensor_type.compare("R") == 0 && ukf.use_radar_)) {

              output_file << sensor_type << " ";
              output_file << estimate(0) << " " << estimate(1) << " " << estimate(2) << " " << estimate(3) << " ";
              output_file << meas_px << " " << meas_py << " ";
              output_file << x_gt << " " << y_gt << " " << vx_gt << " " << vy_gt << " ";
              output_file << RMSE(0) << " " << RMSE(1) << " " << RMSE(2) << " " << RMSE(3) << " ";
              output_file << NIS_radar << " " << NIS_lidar << "\n";
            }
          }
          
          cout << "p_x: " << p_x << " p_y: " << p_y << " NIS-Radar: " << NIS_radar << " NIS-LIDAR: " << NIS_lidar << endl;
          
          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    
  });
  
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h, &output_file](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    // close output file
    if (output_file.is_open()) {
      output_file.close();
    }
    
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
  
  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































