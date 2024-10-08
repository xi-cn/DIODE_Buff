# include "buff_predict/buff_predict_node.h"

namespace Buff
{
  void BuffPredictNode::detectCallback(global_msg::msg::DetectMsg::SharedPtr msg)
  {
    // 添加新的扇页
    track->addFan(msg);
    // 判断新加入的扇页是否需要滤波
    if (track->needFilter()) {
      // 获取滤波扇页
      Fan& fan = track->getFilterFan();
      // 滤波
      cv::Mat z = (cv::Mat_<double>(1, 1) << fan.speed);
      pf->predict();
      cv::Mat estimateState = pf->correct(z);
      // 更新滤波后的速度
      fan.filter_speed = estimateState.at<double>(0, 0);
    }

    // 优化求解
    if (cur_mse > max_mse) {
      cur_mse = all_solver->fit(param, track->getFans());
    }
    else {
      cur_mse = theta_solver->fit(param, track->getFans(), param[0], param[1]);
    }

    // 求解预测点
    predictPoint();
  }

  // 计算预测点
  void BuffPredictNode::predictPoint() {
    // 根据弹速计算延时
    double delay = 0.3;



    // 预测角度差
    double dtheta = - (param[0] / param[1])
                    * (cos(param[1] * (track->getLastTime() + delay) + param[2]) - cos(param[1] * (track->getLastTime()) + param[2]))
                    + (2.090 - param[0]) * delay;

    std::cout << dtheta << "  dtheta\n";
    // 预测点在最新扇页中的偏移
    predict2fan_v = Eigen::Vector3d(sin(dtheta)*0.7, (1-cos(dtheta))*0.7, 0);
    // 预测点在相机坐标系中的偏移
    predict2camera_v = track->getLastFan().buff2camera_q * predict2fan_v + track->getLastFan().buff2camera_v;
    // 预测点在世界坐标系中的偏移
    predict2world_v = track->getLastFan().buff2world_q * predict2fan_v + track->getLastFan().buff2world_v;

  }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Buff::BuffPredictNode>());
    rclcpp::shutdown();
}