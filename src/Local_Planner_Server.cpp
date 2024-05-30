#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "particle_filter/helper_functions.h"
#define PI 3.14159265
using namespace std::chrono_literals;
using std::placeholders::_1;

const bool ANI = false;
const int HEIGHT = 100;
const int WIDTH  = 100;

struct Turtlebot_coord {
	int x;
	int y;
};

class MapServer : public rclcpp::Node
{
  public:
    MapServer()
    : Node("planning_server")
    {
      local_sub1 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_1/real_local", 1, std::bind(&MapServer::real_local_cb1, this, _1));
      local_sub2 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_2/real_local", 1, std::bind(&MapServer::real_local_cb2, this, _1));
      local_sub3 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_3/real_local", 1, std::bind(&MapServer::real_local_cb3, this, _1));
      local_sub4 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_4/real_local", 1, std::bind(&MapServer::real_local_cb4, this, _1));

      obstacle_sub1 = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_1/obstacles", 1, std::bind(&MapServer::obstacle_1_cb, this, _1));
      obstacle_sub2 = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_2/obstacles", 1, std::bind(&MapServer::obstacle_2_cb, this, _1));
      obstacle_sub3 = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_3/obstacles", 1, std::bind(&MapServer::obstacle_3_cb, this, _1));
      obstacle_sub4 = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_4/obstacles", 1, std::bind(&MapServer::obstacle_4_cb, this, _1));

      if(read_map_data(map , HEIGHT, WIDTH)){std::cout << "map 가져옴" << std::endl;}
      else{std::cout << "map 못가져옴" << std::endl;}

      timer_ = this->create_wall_timer(100ms, std::bind(&MapServer::planning, this));
      
      //맵 끌고 오기
    }

    void wait_local(){
      while(turtle_real_x1 == -1) rclcpp::spin_some(this->get_node_base_interface());
    }

  private:
    void planning()
    {
      for(int i = 0; i<HEIGHT; i++){
        for(int j = 0; j<WIDTH; j++){
          grid_map[i][j] = 0;
        }
      }
      for(int i = 0; i<map.landmark_list.size(); i++){
        grid_map[(int)map.landmark_list[i].x_f][(int)map.landmark_list[i].y_f] = 1;
      }

      for(int i = 0; i<HEIGHT; i++){ //turtlebot size
        for(int j = 0; j<WIDTH; j++){
          if(turtle_1_sub && std::sqrt(std::pow(turtle_real_x1-i,2)+std::pow(turtle_real_y1-j,2)) < 5){
            grid_map[i][j] = 1;
          }
          if(turtle_2_sub && std::sqrt(std::pow(turtle_real_x2-i,2)+std::pow(turtle_real_y2-j,2)) < 5){
            grid_map[i][j] = 1;
          }
          if(turtle_3_sub && std::sqrt(std::pow(turtle_real_x3-i,2)+std::pow(turtle_real_y3-j,2)) < 5){
            grid_map[i][j] = 1;
          }
          if(turtle_4_sub && std::sqrt(std::pow(turtle_real_x4-i,2)+std::pow(turtle_real_y4-j,2)) < 5){
            grid_map[i][j] = 1;
          }
        }
      }

      for(int i = 0; i<obstacles_1.size()/2; i++){ //perception obstacles
        grid_map[obstacles_1[2*i]][obstacles_1[2*i+1]] = 7;
      }
      for(int i = 0; i<obstacles_2.size()/2; i++){ //perception obstacles
        grid_map[obstacles_2[2*i]][obstacles_2[2*i+1]] = 7;
      }
      for(int i = 0; i<obstacles_3.size()/2; i++){ //perception obstacles
        grid_map[obstacles_3[2*i]][obstacles_3[2*i+1]] = 7;
      }
      for(int i = 0; i<obstacles_4.size()/2; i++){ //perception obstacles
        grid_map[obstacles_4[2*i]][obstacles_4[2*i+1]] = 7;
      }

      for(int i = 0; i<HEIGHT; i++){
        for(int j = 0; j<WIDTH; j++){
          std::cout << grid_map[i][j] << " ";
        }
        std::cout << std::endl;
      }
    }

    void real_local_cb1(const nav_msgs::msg::Odometry::SharedPtr msg){
      turtle_real_x1 = msg->pose.pose.position.x;
      turtle_real_y1 = msg->pose.pose.position.y;
      turtle_real_heading1 = msg->pose.pose.position.z;

      turtle_1_sub = true;
    }
    void real_local_cb2(const nav_msgs::msg::Odometry::SharedPtr msg){
      turtle_real_x2 = msg->pose.pose.position.x;
      turtle_real_y2 = msg->pose.pose.position.y;
      turtle_real_heading2 = msg->pose.pose.position.z;

      turtle_2_sub = true;
    }
    void real_local_cb3(const nav_msgs::msg::Odometry::SharedPtr msg){
      turtle_real_x3 = msg->pose.pose.position.x;
      turtle_real_y3 = msg->pose.pose.position.y;
      turtle_real_heading3 = msg->pose.pose.position.z;

      turtle_3_sub = true;
    }
    void real_local_cb4(const nav_msgs::msg::Odometry::SharedPtr msg){
      turtle_real_x4 = msg->pose.pose.position.x;
      turtle_real_y4 = msg->pose.pose.position.y;
      turtle_real_heading4 = msg->pose.pose.position.z;

      turtle_4_sub = true;
    }
    void obstacle_1_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      obstacles_1.clear();
      for(int i = 0; i<msg->data.size(); i++){
        obstacles_1.push_back(msg->data[i]);
      }
    }
    void obstacle_2_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      obstacles_2.clear();
      for(int i = 0; i<msg->data.size(); i++){
        obstacles_2.push_back(msg->data[i]);
      }
    }
    void obstacle_3_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      obstacles_3.clear();
      for(int i = 0; i<msg->data.size(); i++){
        obstacles_3.push_back(msg->data[i]);
      }
    }
    void obstacle_4_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      obstacles_4.clear();
      for(int i = 0; i<msg->data.size(); i++){
        obstacles_4.push_back(msg->data[i]);
      }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub3;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub4;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_sub1;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_sub2;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_sub3;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr obstacle_sub4;


    rclcpp::TimerBase::SharedPtr timer_;
    
    int turtle_real_x1 = 50;
    int turtle_real_y1 = 50;
    double turtle_real_heading1 = 0.0;

    int turtle_real_x2 = 50;
    int turtle_real_y2 = 50;
    double turtle_real_heading2 = 0.0;

    int turtle_real_x3 = 50;
    int turtle_real_y3 = 50;
    double turtle_real_heading3 = 0.0;

    int turtle_real_x4 = 50;
    int turtle_real_y4 = 50;
    double turtle_real_heading4 = 0.0;

    bool turtle_1_sub = false;
    bool turtle_2_sub = false;
    bool turtle_3_sub = false;
    bool turtle_4_sub = false;

    std::vector<int> obstacles_1;
    std::vector<int> obstacles_2;
    std::vector<int> obstacles_3;
    std::vector<int> obstacles_4;

    Map map;
    int grid_map[HEIGHT][WIDTH];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto turtle_localization_node = std::make_shared<MapServer>();
  turtle_localization_node->wait_local();
  rclcpp::spin(turtle_localization_node);
  rclcpp::shutdown();

  return 0;
}