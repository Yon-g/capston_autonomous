#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "dstar/Dstar.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "particle_filter/helper_functions.h"

#define PI 3.14159265
using namespace std::chrono_literals;
using std::placeholders::_1;

const bool ANI = false;
const int HEIGHT = 100;
const int WIDTH  = 100;

class Turtle_3_Planning : public rclcpp::Node
{
  public:
    Turtle_3_Planning()
    : Node("planning_1"), ob_map(HEIGHT, std::vector<int>(WIDTH, 0)),  total_map(HEIGHT, std::vector<int>(WIDTH, 0))
    {
      local_sub1 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_1/real_local", 1, std::bind(&Turtle_3_Planning::real_local_cb1, this, _1));
      local_sub2 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_2/real_local", 1, std::bind(&Turtle_3_Planning::real_local_cb2, this, _1));
      local_sub3 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_3/real_local", 1, std::bind(&Turtle_3_Planning::real_local_cb3, this, _1));
      local_sub4 = this->create_subscription<nav_msgs::msg::Odometry>("/turtle_4/real_local", 1, std::bind(&Turtle_3_Planning::real_local_cb4, this, _1));

      obstacle_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/total_obstacles", 1, std::bind(&Turtle_3_Planning::obstacle_cb, this, _1));
      goal_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>("/turtle_3/target_goal", 1, std::bind(&Turtle_3_Planning::goal_cb, this, _1));
     
      path_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/turtle_3/path", 10);
      turtle_map_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/turtle_3/map", 10);

      timer_ = this->create_wall_timer(50ms, std::bind(&Turtle_3_Planning::planning, this));
      
      for (int dx = -robotSize; dx <= robotSize; ++dx) {
          for (int dy = -robotSize; dy <= robotSize; ++dy) {
              if (std::sqrt(dx * dx + dy * dy) >= robotSize - 1 && std::sqrt(dx * dx + dy * dy) <= robotSize + 1) { // 유클리드 거리 검사
                  directions.push_back({dx, dy});
              }
          }
      }
      for (int dx = -robotSize/7*5; dx <= robotSize/7*5; ++dx) {
          for (int dy = -robotSize/7*5; dy <= robotSize/7*5; ++dy) {
              if (std::sqrt(dx * dx + dy * dy) >= robotSize/7*5 - 1 && std::sqrt(dx * dx + dy * dy) <= robotSize/7*5 + 1) { // 유클리드 거리 검사
                  map_directions.push_back({dx, dy});
              }
          }
      }
      //맵 끌고 오기
      if(read_map_data_planning(map , HEIGHT, WIDTH)){std::cout << "map 가져옴" << std::endl;}
      else{std::cout << "map 못가져옴" << std::endl;}
    }

    void wait_local(){
      while(turtle_real_x3 == -1) rclcpp::spin_some(this->get_node_base_interface());
    }

  private:
    void planning()
    {
      std::vector<std::vector<int>> temp_map(HEIGHT, std::vector<int>(WIDTH, 0));

      for (int i = 0; i < HEIGHT; i++) {
          for (int j = 0; j < WIDTH; j++) {
              if (ob_map[i][j] == 2) {
                  for (const auto& dir : directions) {
                      int newX = i + dir.first;
                      int newY = j + dir.second;
                      if (newX >= 0 && newX < HEIGHT && newY >= 0 && newY < WIDTH) {
                          temp_map[newX][newY] = 1; // 장애물 추가
                          
                      }
                  }
              }
              else if (ob_map[i][j] == 1) {
                  for (const auto& dir : map_directions) {
                      int newX = i + dir.first;
                      int newY = j + dir.second;
                      if (newX >= 0 && newX < HEIGHT && newY >= 0 && newY < WIDTH) {
                          temp_map[newX][newY] = 1; // 장애물 추가
                          
                      }
                  }
              }
          }
      }
      
      if(turtle_1_sub) {
        for (int i = -12; i <= 12; ++i) {
          for (int j = -12; j <= 12; ++j) {
            int nx = turtle_real_x1 + round(i * cos(turtle_real_heading1) - j * sin(turtle_real_heading1));
            int ny = turtle_real_y1 + round(i * sin(turtle_real_heading1) + j * cos(turtle_real_heading1));
            if (nx >= 0 && nx < HEIGHT && ny >= 0 && ny < WIDTH) {
                temp_map[nx][ny] = 1;
            }
          }
        }
      }
      if(turtle_2_sub) {
        for (int i = -12; i <= 12; ++i) {
          for (int j = -12; j <= 12; ++j) {
            int nx = turtle_real_x2 + round(i * cos(turtle_real_heading2) - j * sin(turtle_real_heading2));
            int ny = turtle_real_y2 + round(i * sin(turtle_real_heading2) + j * cos(turtle_real_heading2));
            if (nx >= 0 && nx < HEIGHT && ny >= 0 && ny < WIDTH) {
                temp_map[nx][ny] = 1;
            }
          }
        }
      }
      if(turtle_4_sub) {
        for (int i = -12; i <= 12; ++i) {
          for (int j = -12; j <= 12; ++j) {
            int nx = turtle_real_x4 + round(i * cos(turtle_real_heading4) - j * sin(turtle_real_heading4));
            int ny = turtle_real_y4 + round(i * sin(turtle_real_heading4) + j * cos(turtle_real_heading4));
            if (nx >= 0 && nx < HEIGHT && ny >= 0 && ny < WIDTH) {
                temp_map[nx][ny] = 1;
            }
          }
        }
      }


      int d_ob;
      for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
          d_ob = temp_map[i][j] - total_map[i][j];

          if(d_ob == -1) dstar.updateCell(i, j, 1);
          else if(d_ob == 1) dstar.updateCell(i, j, -1);
        }
      } 


      total_map = temp_map;

      auto map_array = std_msgs::msg::Int32MultiArray();

      for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
          if(total_map[i][j] == 1){
            map_array.data.push_back(i);
            map_array.data.push_back(j);
          }
        }
      } 

      turtle_map_pub->publish(map_array);

      if(path_sub){
        dstar.replan();
        std::list<state> path = dstar.getPath();

        if(path.size() == 0){
          initial_dstar(turtle_real_x3,turtle_real_y3,path_goal_x,path_goal_y);
        }
        auto path_array = std_msgs::msg::Int32MultiArray();

        for (const auto& s : path) {
          path_array.data.push_back(s.x);
          path_array.data.push_back(s.y);
        }
        if (!path.empty()) {
          state last_state = path.back();
          if (last_state.x == path_goal_x && last_state.y == path_goal_y) {
            // 마지막 좌표가 goal 좌표와 동일함
            std::cout << "Last coordinate matches the goal coordinate." << std::endl;
          } else {
            return;
          }
        } else {
          // path가 비어있음
          std::cout << "Path is empty." << std::endl;
        }
        path_pub->publish(path_array);
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

    void obstacle_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
      for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
          ob_map[i][j] = 0;
        }
      }
      for(int i = 0; i<map.landmark_list.size(); i++){
        if(map.landmark_list[i].id_i == 1) ob_map[map.landmark_list[i].x_f][map.landmark_list[i].y_f] = 1;
        if(map.landmark_list[i].id_i == 2) ob_map[map.landmark_list[i].x_f][map.landmark_list[i].y_f] = 2;
      }
      for(int i = 0; i<msg->data.size()/3; i++){
        if(msg->data[3*i+2] < 5.0) continue;
        ob_map[msg->data[3*i]][msg->data[3*i+1]] = 1;
      }
    }

    void initial_dstar(int start_x, int start_y, int goal_x, int goal_y){
      if(total_map[start_x][start_y] == 1){
        float dist = 0;
        float dist_small = 1000;
        int start_v_x;
        int start_v_y;
        for (int i = 0; i < HEIGHT; i++) {
          for (int j = 0; j < WIDTH; j++) {
            if(total_map[i][j] == 0){
              dist = sqrt(pow(start_x-i,2)+pow(start_y-j,2));
              if(dist < dist_small){
                // std::cout << dist << std::endl;
                // std::cout << i << "," << j << std::endl;
                dist_small = dist;
                start_v_x = i;
                start_v_y = j;
              }
            }
          }
        }
        std::cout << "edit : " << start_v_x << "," << start_v_y << " and " << goal_x << "," << goal_y << std::endl; 

        dstar.init(start_v_x, start_v_y, goal_x, goal_y);
      }
      else{
        std::cout << "no edit : " << start_x << "," << start_y << " and " << goal_x << "," << goal_y << std::endl; 

        dstar.init(start_x, start_y, goal_x, goal_y);
      }
      for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
          total_map[i][j] = 0;
        }
      }
    }

    void goal_cb(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
      if(path_start_x == msg->data[0] && path_start_y == msg->data[1] && path_goal_x == msg->data[2] &&path_goal_y == msg->data[3]){
        path_change = false;
      }
      else{
        path_change = true;
      }
      path_start_x = msg->data[0];
      path_start_y = msg->data[1];
      path_goal_x = msg->data[2];
      path_goal_y = msg->data[3];
      if(path_change){
        initial_dstar(path_start_x,path_start_y,path_goal_x,path_goal_y);
        std::cout << "path initial" << std::endl;
      }
      path_sub = true;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub1;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub2;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub3;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_sub4;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr goal_sub;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr path_pub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr turtle_map_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    
    int path_goal_x = -1.0;
    int path_goal_y = -1.0;

    int path_start_x = -1.0;
    int path_start_y = -1.0;

    int robotSize = 7;

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
    bool path_sub = false;
    bool path_change = false;

    std::vector<std::vector<int>> ob_map;
    std::vector<std::vector<int>> total_map;

    std::vector<std::pair<int, int>> directions;
    std::vector<std::pair<int, int>> map_directions;
    Map map;
    Dstar dstar;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto turtle_localization_node = std::make_shared<Turtle_3_Planning>();
  turtle_localization_node->wait_local();
  rclcpp::spin(turtle_localization_node);
  rclcpp::shutdown();

  return 0;
}