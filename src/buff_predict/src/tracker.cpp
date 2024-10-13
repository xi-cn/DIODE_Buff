# include "buff_predict/tracker.h"

namespace Buff
{
  // geometry消息格式转eigen
  Eigen::Quaterniond Track::geometry2eigen(geometry_msgs::msg::Quaternion q)
  {
    Eigen::Quaterniond eigen_q(q.w, q.x, q.y, q.z);
    return eigen_q;
  }
  Eigen::Vector3d Track::geometry2eigen(geometry_msgs::msg::Point v)
  {
    Eigen::Vector3d eigen_v(v.x, v.y, v.z);
    return eigen_v;
  }

  Track::Track(int max_delt_time_, double max_delt_angle_)
  {
    this->max_delt_time = max_delt_time_;
    this->max_delt_angle = max_delt_angle_;
  }

  // 添加新的扇页
  void Track::addFan(global_msg::msg::DetectMsg::SharedPtr msg)
  {
    Fan fan;
    // 时间
    fan.time_ = msg->timestamp;
    fan.second = double(fan.time_) / 1e9;
    // 坐标转换关系
    fan.buff2camera_q = geometry2eigen(msg->buff2camera_q);
    fan.buff2camera_v = geometry2eigen(msg->buff2camera_v);
    fan.camera2world_q = geometry2eigen(msg->camera2world_q);
    fan.camera2world_v = geometry2eigen(msg->camera2world_v);
    fan.buff2world_q = geometry2eigen(msg->buff2world_q);
    fan.buff2world_v = geometry2eigen(msg->buff2world_v);
    
    // 当前队列为空
    if (fans.size() == 0) {
      fan.delt_time = 0;
      fan.delt_second = 0;
      fan.delt_angle = -1;
      fan.speed = -1;
      fan.filter_speed = -1;
      // 直接加入新的扇页
      fans.push_back(fan);
    }
    // 不为空
    else {
      // 时间差
      fan.delt_time = fan.time_ - fans.back().time_;
      fan.delt_second = double(fan.delt_time) / 1e9;
      // 角度差
      Eigen::Quaterniond delt_q = fans.back().buff2camera_q.inverse() * fan.buff2camera_q;
      fan.delt_angle = Eigen::AngleAxisd(delt_q).angle();
      // 角度大于阈值
      if (fan.delt_angle > max_delt_angle) {
        fan.speed = -1;
        fan.filter_speed = -1;
      }
      // 角度合适
      else {
        fan.speed = fan.delt_angle / fan.delt_second;
      }

      fans.push_back(fan);
    }
  }
    
  

  // 判断是否需要滤波
  bool Track::needFilter()
  {
    if (fans.size() == 0) {
      return false;
    }
    else {
      return fans.back().speed != -1;
    }
  }

  // 维护扇页队列
  void Track::updateFans(uint64_t now)
  {
    while (!fans.empty()) {
      if (now / 1e9 - fans.front().second > max_delt_time) {
        fans.pop_front();
      }
      else {
        break;
      }
    }
  }

  // 获取最近的时间
  double Track::getLastTime() {
    if (fans.empty()) {
      return 0;
    }
    else {
      return fans.back().second;
    }
  }
  
} // namespace Buff
