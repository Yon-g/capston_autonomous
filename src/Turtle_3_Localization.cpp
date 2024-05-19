#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <iomanip>
#include <random>

#include "particle_filter/particle_filter.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#define PI 3.14159265
using namespace std::chrono_literals;
using std::placeholders::_1;

const bool ANI = false;
const int HEIGHT = 100;
const int WIDTH  = 100;

class Localization : public rclcpp::Node
{
  public:
    Localization()
    : Node("local_node3")
    {
      auto qos = rclcpp::SensorDataQoS();
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/turtle_3/cmd_vel", 1, std::bind(&Localization::cmd_vel_cb, this, _1));
      local_sub = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_3/dm_local", 1, std::bind(&Localization::local_cb, this, _1));
      lidar_sensor_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/turtle_3/scan", qos, std::bind(&Localization::lidar_sensor_cb, this, _1));
      local_pub = this->create_publisher<nav_msgs::msg::Odometry>("/turtle_3/real_local", 10);
      obstacle_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/turtle_3/obstacles", 10);


      if(read_map_data(map , HEIGHT, WIDTH)){std::cout << "map 가져옴" << std::endl;}
      else{std::cout << "map 못가져옴" << std::endl;}

      timer_ = this->create_wall_timer(200ms, std::bind(&Localization::particle_filtering, this));
      timer2_ = this->create_wall_timer(200ms, std::bind(&Localization::obstacle_compute, this));
    }

    void wait_local(){
      while(turtle_real_x == -1) rclcpp::spin_some(this->get_node_base_interface());
      local_sub_bool = true;
    }

  private:
    void obstacle_compute()
    {
      auto obstacles = std_msgs::msg::Int32MultiArray();
      for(int j=0; j<lidar_observation.size(); ++j){
        LandmarkObs current_obs = lidar_observation[j];
        if(sqrt(pow(current_obs.x,2)+pow(current_obs.y,2)) < 24){
          obstacles.data.push_back(static_cast<int>((current_obs.x * cos(turtle_real_heading)) - (current_obs.y * sin(turtle_real_heading)) + turtle_real_x));
          obstacles.data.push_back(static_cast<int>((current_obs.x * sin(turtle_real_heading)) + (current_obs.y * cos(turtle_real_heading)) + turtle_real_y));
        }   
      }
      obstacle_pub->publish(obstacles);
    }
    void particle_filtering()
    {
      // Initialize particle filter if this is the first time step.
      if (!pf.initialized()) {
        n_x = N_x_init(gen);
        n_y = N_y_init(gen);
        n_theta = N_theta_init(gen);
        pf.init(turtle_real_x + n_x, turtle_real_y + n_y, turtle_real_heading + n_theta, sigma_pos); //particle 생성
      }
      else {
        // Predict the vehicle's next state (noiseless).
        pf.prediction(delta_t, sigma_pos, linear_x, angular_z); //velocity,yawrate로 이동 위치 추정
      }
      // simulate the addition of noise to noiseless observation data.

      // Update the weights and resample
      pf.updateWeights(sensor_range, sigma_landmark, lidar_observation, map); //particle 가중치 업데이트
      pf.resample(); //particle 재생성
      
      auto turtlebot_predict_pose = pf.estimate();
      
      turtle_real_x = turtlebot_predict_pose.first[0];
      turtle_real_y = turtlebot_predict_pose.first[1];
      turtle_real_heading = turtlebot_predict_pose.first[2];
      std::cout << "x:" << (double)(turtle_real_x) << std::endl;
      std::cout << "y:" << (double)(turtle_real_y) << std::endl;
      std::cout << "heading:" <<turtle_real_heading << std::endl;

      auto odom = nav_msgs::msg::Odometry();
      odom.pose.pose.position.x = turtle_real_x;
      odom.pose.pose.position.y = turtle_real_y;
      odom.pose.pose.position.z = turtle_real_heading;
      local_pub->publish(odom);
    }

    bool check_range_in_turtlebot(double theta, float range){ // 의자 다리 인식 안하게
      float x = range*cos(theta);
      float y = range*sin(theta);
      if(x<0.12 && x >-0.12 && y<0.12 && y >-0.12){
        return false;
      }
      else{
        return true;
      }
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      linear_x = msg->linear.x;
      angular_z = msg->angular.z;
    }

    void lidar_sensor_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      lidar_observation.clear();
      for(int i = 0; i <360; i++){
        double theta = i*PI/180;
        if(msg->ranges[i] >3){
          continue;
        }
        if(check_range_in_turtlebot(theta,(float)msg->ranges[i])){
          LandmarkObs ob;
          ob.x = (int)(msg->ranges[i]*100/3*cos(i*PI/180));
          ob.y = (int)(msg->ranges[i]*100/3*sin(i*PI/180));
          ob.id = i;
          lidar_observation.push_back(ob);
        }
      }
    }

    void local_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
      if(!local_sub_bool){
        turtle_real_x = (int)(msg->pose.pose.position.x * 100/3);
        turtle_real_y = (int)(msg->pose.pose.position.y * 100/3);
      }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sensor_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_pub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;

    ParticleFilter pf;

    bool local_sub_bool = false;
    
    int turtle_real_x = 50;
    int turtle_real_y = 50;
    double turtle_real_heading = 0.0;
  
    double linear_x = 0.0;
    double angular_z = 0.0;

    double delta_t = 0.2; // Time elapsed between measurements [sec]
    double sensor_range = 50; // Sensor range [m]
    
    double sigma_pos [3] = {0.5, 0.5, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark [2] = {0.5, 0.5}; // Landmark measurement uncertainty [x [m], y [m]]

    std::default_random_engine gen;
    std::normal_distribution<double> N_x_init{0, sigma_pos[0]};
    std::normal_distribution<double> N_y_init{0, sigma_pos[1]};
    std::normal_distribution<double> N_theta_init{0, sigma_pos[2]};
    std::normal_distribution<double> N_obs_x{0, sigma_landmark[0]};
    std::normal_distribution<double> N_obs_y{0, sigma_landmark[1]};
    double n_x, n_y, n_theta, n_range, n_heading;
    // Read map data
    Map map;
    
    std::vector<LandmarkObs> lidar_observation;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto turtle_localization_node = std::make_shared<Localization>();
  turtle_localization_node->wait_local();
  rclcpp::spin(turtle_localization_node);
  rclcpp::shutdown();

  return 0;
}