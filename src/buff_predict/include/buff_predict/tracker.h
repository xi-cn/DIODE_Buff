# ifndef TRACK
# define TRACK

# include <Eigen/Core>
# include <Eigen/Geometry>
# include <list>
# include <global_msg/msg/detect_msg.hpp>
# include <iostream>

namespace Buff
{
  struct Fan
  {
    uint32_t time_;
    uint32_t delt_time;
    double second;
    double delt_second;
    double delt_angle;
    double speed;
    double filter_speed;
    Eigen::Quaterniond buff2camera_q;
    Eigen::Vector3d buff2camera_v;
    Eigen::Quaterniond camera2world_q;
    Eigen::Vector3d camera2world_v;
    Eigen::Quaterniond buff2world_q;
    Eigen::Vector3d buff2world_v;
  };
  

  class Track
  {
  public:
    Track(int max_delt_time_, double max_delt_angle_);
    // 添加新的扇页
    void addFan(global_msg::msg::DetectMsg::SharedPtr msg);
    // 判断是否需要滤波
    bool needFilter();
    // 获取要滤波的扇页
    Fan& getFilterFan() {return fans.back();}
    // 维护扇页队列
    void updateFans(uint32_t now);
    // 获取扇页队列
    std::list<Fan>& getFans() {return fans;}
    // 获取最近的时间
    double getLastTime();
    // 获取最新的扇页
    Fan getLastFan() {return fans.back();}
  private:
    // geometry消息格式转eigen
    Eigen::Quaterniond geometry2eigen(geometry_msgs::msg::Quaternion q);
    Eigen::Vector3d geometry2eigen(geometry_msgs::msg::Point v);

    // 扇页队列
    std::list<Fan> fans;
    // 扇页队列最长时间跨度
    int max_delt_time;
    // 两次扇页间最大角度差
    double max_delt_angle;
  };
}

# endif