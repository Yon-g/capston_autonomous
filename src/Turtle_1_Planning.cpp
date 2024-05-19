#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "particle_filter/helper_functions.h"
#include "astar/AStar.hpp"

#define PI 3.14159265
using namespace std::chrono_literals;
using std::placeholders::_1;

const bool ANI = false;
const int HEIGHT = 100;
const int WIDTH  = 100;

class Turtle_1_Planning : public rclcpp::Node
{
  public:
    Turtle_1_Planning()
    : Node("real_time_map_node")
    {
      local_sub1 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_1/real_local", 1, std::bind(&Turtle_1_Planning::real_local_cb1, this, _1));
      local_sub2 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_2/real_local", 1, std::bind(&Turtle_1_Planning::real_local_cb2, this, _1));
      local_sub3 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_3/real_local", 1, std::bind(&Turtle_1_Planning::real_local_cb3, this, _1));
      local_sub4 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_4/real_local", 1, std::bind(&Turtle_1_Planning::real_local_cb4, this, _1));

      obstacle_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/total_obstacles", 1, std::bind(&Turtle_1_Planning::obstacle_cb, this, _1));
      goal_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_1/target_goal", 1, std::bind(&Turtle_1_Planning::goal_cb, this, _1));

      path_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/turtle_1/path", 10);


      timer_ = this->create_wall_timer(50ms, std::bind(&Turtle_1_Planning::planning, this));
      
      generator.setWorldSize({HEIGHT, WIDTH});
      generator.setDiagonalMovement(true);
      generator.setHeuristic(AStar::Heuristic::euclidean);
      //맵 끌고 오기
    }

    void wait_local(){
      while(turtle_real_x1 == -1) rclcpp::spin_some(this->get_node_base_interface());
    }

  private:
    void planning()
    {
      generator.clearCollisions();
      auto start = std::chrono::high_resolution_clock::now(); // 시작 시간 기록
      std::vector<std::vector<int>> map(HEIGHT, std::vector<int>(WIDTH, 0));

      for(int i = 0; i<obstacle_map.landmark_list.size(); i++){
        map[obstacle_map.landmark_list[i].x_f][obstacle_map.landmark_list[i].y_f] = 1;
      }
      if(turtle_2_sub) map[turtle_real_x2][turtle_real_y2] = 1;
      if(turtle_3_sub) map[turtle_real_x3][turtle_real_y3] = 1;
      if(turtle_4_sub) map[turtle_real_x4][turtle_real_y4] = 1;

      std::vector<std::vector<int>> tempMap = map;

      std::vector<std::pair<int, int>> directions;
      for (int dx = -robotSize; dx <= robotSize; ++dx) {
        for (int dy = -robotSize; dy <= robotSize; ++dy) {
          if (std::sqrt(dx * dx + dy * dy) >= robotSize - 0.5 && std::sqrt(dx * dx + dy * dy) <= robotSize + 0.5) { // 유클리드 거리 검사
            directions.push_back({dx, dy});
          }
        }
      }

      for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
          if (map[i][j] == 1) {
            for (const auto& dir : directions) {
              int newX = i + dir.first;
              int newY = j + dir.second;
              if (newX >= 0 && newX < HEIGHT && newY >= 0 && newY < WIDTH) {
                tempMap[newX][newY] = 1;
              }
            }
          }
        }
      } 

      for (int i = 0; i < HEIGHT; ++i) {
        for (int j = 0; j < WIDTH; ++j) {
          if(tempMap[i][j] == 1){
            generator.addCollision({i,j});
          }
        }
      }

      auto path = generator.findPath({20, 30}, {path_goal_x, path_goal_y});

      auto path_array = std_msgs::msg::Int32MultiArray();

      for(auto& coordinate : path) {
        path_array.data.push_back(coordinate.x);
        path_array.data.push_back(coordinate.y);
      }

      path_pub->publish(path_array);

      auto end = std::chrono::high_resolution_clock::now(); // 끝 시간 기록
      std::chrono::duration<double> elapsed = end - start; // 경과 시간 계산

      std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl; // 경과 시간 출력
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

    void obstacle_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
      obstacle_map.landmark_list.clear();
      for(int i = 0; i<msg->data.size()/3; i++){
        Map::single_landmark_s ob;
        if(msg->data[3*i+2] < 1.0) continue;
        ob.x_f = msg->data[3*i];
        ob.y_f = msg->data[3*i+1];
        obstacle_map.landmark_list.push_back(ob);
      }
    }

    void goal_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      path_goal_x = msg->data[0];
      path_goal_y = msg->data[1];
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub3;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub4;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_sub;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    
    int path_goal_x = 70;
    int path_goal_y = 70;

    int robotSize = 10;

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

    AStar::Generator generator;
    Map obstacle_map;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto turtle_localization_node = std::make_shared<Turtle_1_Planning>();
  turtle_localization_node->wait_local();
  rclcpp::spin(turtle_localization_node);
  rclcpp::shutdown();

  return 0;
}