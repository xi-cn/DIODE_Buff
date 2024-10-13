# include "buff_predict/buff_predict_node.h"

namespace Buff
{
  void BuffPredictNode::detectCallback(global_msg::msg::DetectMsg::SharedPtr msg)
  {
    // 维护扇页队列 移除旧的扇页
    track->updateFans(this->get_clock()->now().nanoseconds());

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
      
      std::cout << fan.time_ << "\n";
      std::cout << fan.second << "\n\n";
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

    // 可视化展示
    if (true) {
      showPredictResult(msg);
      // drawPredictCurve();
    }
    
  }

  // 计算预测点
  void BuffPredictNode::predictPoint() {
    // 根据弹速计算延时
    double delay = 0.15;



    // 预测角度差
    double dtheta = - (param[0] / param[1])
                    * (cos(param[1] * (track->getLastTime() + delay) + param[2]) - cos(param[1] * (track->getLastTime()) + param[2]))
                    + (2.090 - param[0]) * delay;

    // 预测点在最新扇页中的偏移
    predict2fan_v = Eigen::Vector3d(sin(dtheta)*0.7, (1-cos(dtheta))*0.7, 0);
    // 预测点在相机坐标系中的偏移
    predict2camera_v = track->getLastFan().buff2camera_q * predict2fan_v + track->getLastFan().buff2camera_v;
    // 预测点在世界坐标系中的偏移
    predict2world_v = track->getLastFan().buff2world_q * predict2fan_v + track->getLastFan().buff2world_v;

  }

  // 可视化展示
  void BuffPredictNode::showPredictResult(global_msg::msg::DetectMsg::SharedPtr msg)
  {
    int height = msg->src.height;  
    int width = msg->src.width;  
    int step_ = msg->src.step; 

    // 将图像转换为bgr8格式
    cv::Mat raw_image(height, width, CV_8UC3, const_cast<uint8_t*>(msg->src.data.data()), step_); 

    cv::Mat centerPoint = (cv::Mat_<double>(1, 3) << predict2camera_v.x(), predict2camera_v.y(), predict2camera_v.z());
    cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64F);

    // 将相机中心映射到图片上
    cv::Point p = pnp->projectPoints(centerPoint, rvec, tvec);

    cv::circle(raw_image, p, 10, cv::Scalar(180, 90, 100), cv::FILLED);
    cv::resize(raw_image, raw_image, cv::Size(raw_image.cols/2, raw_image.rows/2));
    // 展示图像
    cv::imshow("detect image", raw_image);
    cv::waitKey(5);
  }

  // 绘制预测曲线
  void BuffPredictNode::drawPredictCurve()
  {
    // 获取时间和速度
    std::vector<double> time_recorder;
    std::vector<double> speed_recorder;
    std::vector<double> filter_speed_recorder;
    std::vector<double> fit_speed_recorder;
    for (auto iter = track->getFans().begin(); iter != track->getFans().end(); iter++) {
      time_recorder.push_back(iter->second);
      speed_recorder.push_back(iter->speed);
      filter_speed_recorder.push_back(iter->filter_speed);
    }
    fit_speed_recorder.resize(time_recorder.size());
    for (int i = 0; i < fit_speed_recorder.size(); ++i) {
        fit_speed_recorder[i] = param[0]*sin(param[1]*time_recorder[i] + param[2]) + 2.090 - param[0];
    }

    matplotlibcpp::clf();
    matplotlibcpp::subplot2grid(2, 1, 0);
    matplotlibcpp::scatter(time_recorder, speed_recorder);
    matplotlibcpp::ylim(0, 3);

    matplotlibcpp::subplot2grid(2, 1, 1);
    matplotlibcpp::scatter(time_recorder, filter_speed_recorder);
    matplotlibcpp::plot(time_recorder, fit_speed_recorder);
    matplotlibcpp::ylim(0, 3);

    matplotlibcpp::pause(0.001);
  }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Buff::BuffPredictNode>());
    rclcpp::shutdown();
}