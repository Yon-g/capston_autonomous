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
#include "std_msgs/msg/float32_multi_array.hpp"
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
    : Node("local_node4")
    {
      auto qos = rclcpp::SensorDataQoS();
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/turtle_4/cmd_vel", 1, std::bind(&Localization::cmd_vel_cb, this, _1));
      local_sub = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_4/dm_local", 1, std::bind(&Localization::local_cb, this, _1));
      lidar_sensor_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/turtle_4/scan", qos, std::bind(&Localization::lidar_sensor_cb, this, _1));
      dwm1000_sub = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_4/dwm1000", qos, std::bind(&Localization::dwm1000_cb, this, _1));

      // map_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/tortal_map", qos, std::bind(&Localization::map_cb, this, _1));
      local_pub = this->create_publisher<nav_msgs::msg::Odometry>("/turtle_4/real_local", 10);
      obstacle_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/turtle_4/obstacles", 10);


      if(read_map_data(map , HEIGHT, WIDTH)){std::cout << "map 가져옴" << std::endl;}
      else{std::cout << "map 못가져옴" << std::endl;}

      timer_ = this->create_wall_timer(50ms, std::bind(&Localization::particle_filtering, this));
      timer2_ = this->create_wall_timer(100ms, std::bind(&Localization::obstacle_compute, this));
    }

    void wait_local(){
      while(turtle_real_x == -1) rclcpp::spin_some(this->get_node_base_interface());
      local_sub_bool = true;
    }

  private:
    void obstacle_compute()
    {
      auto obstacles = std_msgs::msg::Int32MultiArray();
      for(int j=0; j<lidar_obstacle.size(); ++j){
        LandmarkObs current_obs = lidar_obstacle[j];
        // if(sqrt(pow(current_obs.x,2)+pow(current_obs.y,2)) < 24){
          obstacles.data.push_back(static_cast<int>((current_obs.x * cos(turtle_real_heading)) - (current_obs.y * sin(turtle_real_heading)) + turtle_real_x));
          obstacles.data.push_back(static_cast<int>((current_obs.x * sin(turtle_real_heading)) + (current_obs.y * cos(turtle_real_heading)) + turtle_real_y));
        // }   
      }
      obstacle_pub->publish(obstacles);
    }
    void particle_filtering()
    {
      if(dwm1000_x != 0 && sqrt(pow(dwm1000_x-turtle_real_x,2)+pow(dwm1000_y-turtle_real_y,2)) > 40){
        n_x = N_x_init(gen);
        n_y = N_y_init(gen);
        n_theta = N_theta_init(gen);
        pf.init(dwm1000_x + n_x, dwm1000_y + n_y, turtle_real_heading + n_theta, sigma_pos);
      }
      else{
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
      if(x<0.15 && x >-0.15 && y<0.15 && y >-0.15){
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
      lidar_obstacle.clear();
      for(int i = 0; i <360; i++){
        double theta = i*PI/180;
        if(msg->ranges[i] >3 || msg->ranges[i] == 0){
          continue;
        }
        if(check_range_in_turtlebot(theta,(float)msg->ranges[i])){
          LandmarkObs ob;
          ob.x = (int)(msg->ranges[i]*100/3*cos(i*PI/180));
          ob.y = (int)(msg->ranges[i]*100/3*sin(i*PI/180));
          ob.id = i;
          int x = ((ob.x * cos(turtle_real_heading)) - (ob.y * sin(turtle_real_heading)) + turtle_real_x);
          int y = ((ob.x * sin(turtle_real_heading)) + (ob.y * cos(turtle_real_heading)) + turtle_real_y);
          if(x >=9 && x <=91 && y >= 14 && y <=85){
            lidar_obstacle.push_back(ob);
          }else{
            lidar_observation.push_back(ob);
          }
        }
      }
    }

    void local_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
      if(!local_sub_bool){
        turtle_real_x = (int)(msg->pose.pose.position.x * 100/3);
        turtle_real_y = (int)(msg->pose.pose.position.y * 100/3);
      }
    }
    void dwm1000_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
      std::cout << "dwm1000_x "<<dwm1000_x << std::endl;
      std::cout << "dwm1000_y "<<dwm1000_y << std::endl;

      dwm1000_x = (int)(msg->pose.pose.position.x * 100/3);
      dwm1000_y = (int)(msg->pose.pose.position.y * 100/3);
    }
    // void map_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    //   map.landmark_list.clear();
    //   for(int i = 0; i< msg->data.size()/3; i++){
    //     if(msg->data[3*i+2] > 10){
    //       Map::single_landmark_s map_particle;
    //       map_particle.x_f = msg->data[3*i];
    //       map_particle.y_f = msg->data[3*i+1];
    //       map.landmark_list.push_back(map_particle);
    //     }
    //   }
    // }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sensor_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dwm1000_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr map_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_pub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;

    ParticleFilter pf;

    bool local_sub_bool = false;

    int dwm1000_x = 0;
    int dwm1000_y = 0;
    
    int turtle_real_x = 20;
    int turtle_real_y = 80;
    double turtle_real_heading = 0.0;
  
    double linear_x = 0.0;
    double angular_z = 0.0;

    double delta_t = 0.05; // Time elapsed between measurements [sec]
    double sensor_range = 50; // Sensor range [m]
    
    double sigma_pos [3] = {0.5, 0.5, 0.5}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
    double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

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
    std::vector<LandmarkObs> lidar_obstacle;
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