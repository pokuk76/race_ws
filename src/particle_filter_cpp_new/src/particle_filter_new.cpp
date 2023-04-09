#include <cstdio>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <experimental/random>
#include <cmath>

#include <includes_for_pf_cpp.h>
#include <RangeLib.h>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#define MAX_DISTANCE 500
#define THETA_DISC 108
#define MB (1024.0*1024.0)
#define MAX_PARTICLES 
#define MOTION_DISPERSION_X
#define MOTION_DISPERSION_y
#define MOTION_DISPERSION_THETA

#define Q(x) #x
#define QUOTE(x) Q(x)

#define GRID_STEP 10
#define GRID_RAYS 40
#define GRID_SAMPLES 1
#define RANDOM_SAMPLES 200000

// using namespace ranges;
using namespace std;
using namespace Eigen;

using namespace std::chrono_literals;


float quaternion2angle(tf2::Quaternion q);
Eigen::Matrix2f rotationMatrix(float theta);
Eigen::MatrixXf map_to_world(Eigen::MatrixXf poses, float resolution, float x_origin, float y_origin, float angle);


class ParticleFilter : public rclcpp::Node
{
  /*
    Add Class Variables 
    MatrixXf this->particles(this->MAX_PARTICLES,3);
  */
  public:
    std::random_device rd;                          //Seed for random number generator
    std::default_random_engine generator;
    
    // auto map_msg;
    float resolution;
    int width;
    int height;
    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    int MAX_RANGE_PX;
    
    bool first_sensor_update = 1;
    // int MAX_RANGE_PX = 0;
    typedef Matrix<float, Dynamic, 1> VectorXf;
    VectorXf odometry_data;// = {{local_delta(0),local_delta(1),orientation - self.last_pose(2)}};
    VectorXf laser_angles;
    VectorXf downsampled_angles;
    Vector3f last_pose;
    MatrixXf local_deltas;
    MatrixXf queries;
    VectorXf ranges;
    MatrixXf sensor_model_table;
    MatrixXf particles;
    VectorXf inferred_pose;
    VectorXf weights;
    MatrixXf permissible_region;

    // RayMarchingGPU *range_method;
    // Map Service Client
    // auto omap_client_ = create_client<nav_msgs::srv::GetMap>("/map_server/map");
    // get_omap();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exited map client");


    // precompute_sensor_model();


    // initialize_global();
    
      ParticleFilter() : Node("particle_filter")
      {
      }
    
    private:
    

 void get_omap()
{
   auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
   using ServiceResponseFuture =
   rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;
   auto response_received_callback = [this](ServiceResponseFuture future)
    {
   	auto result = future.get();
   	rclcpp::shutdown();
    };
   auto future_result = omap_client_->async_send_request(request, response_received_callback);
   auto map_msg = future_result.get()->map;
   this->resolution = future_result.get()->map.info.resolution;
   this->width = future_result.get()->map.info.width;
   this->height = future_result.get()->map.info.height;
   cout << "resolution: " << this->resolution << endl;
   cout << "width: " << this->width << endl;
   cout << "height: " << this->height << endl;
   //this->orientation_x = future_result.get()->map.pose.pose.orientation.x;
   //this->orientation_y = future_result.get()->map.pose.pose.orientation.y;
   //this->orientation_z = future_result.get()->map.pose.pose.orientation.z;
   //this->orientation_w = future_result.get()->map.pose.pose.orientation.w;

}

    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr omap_client_;
    

    
};


int main(int argc, char ** argv)
{
  cout << "hello world particle_filter_cpp package" << endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilter>());
  rclcpp::shutdown();
  return 0;
}
