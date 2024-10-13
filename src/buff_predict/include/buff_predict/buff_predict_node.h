# ifndef BUFF_PREDICT_NODE
# define BUFF_PREDICT_NODE

# include <rclcpp/rclcpp.hpp>
# include <global_msg/msg/detect_msg.hpp>
# include <cv_bridge/cv_bridge.h>
# include <opencv2/opencv.hpp>
# include <vector>
# include <buff_predict/matplotlibcpp.h>

# include "buff_predict/kalman.h"
# include "buff_predict/solver.h"
# include "buff_predict/tracker.h"
# include "buff_detect/pnp.h"

namespace Buff
{
  class BuffPredictNode : public rclcpp::Node
  {
  public:
    BuffPredictNode() : Node("buff_predict_node")
    {
      detect_sub = this->create_subscription<global_msg::msg::DetectMsg>(
        "/buff_detect", 10, std::bind(&BuffPredictNode::detectCallback, this, std::placeholders::_1)
      );

      track = std::make_shared<Track>(10, M_PI * 0.4);
      pf = std::make_shared<Kalman>();
      all_solver = std::make_shared<AllParamSolver>();
      theta_solver = std::make_shared<ThetaSolver>();
      param = new double[3]{0, 0, 0};
      pnp = std::make_shared<PnP>("");
    }
  
  private:
    // 视觉检测消息回调函数
    void detectCallback(global_msg::msg::DetectMsg::SharedPtr msg);
    // 计算预测点
    void predictPoint();
    // 可视化展示
    void showPredictResult(global_msg::msg::DetectMsg::SharedPtr msg);
    // 绘制预测曲线
    void drawPredictCurve();

    rclcpp::Subscription<global_msg::msg::DetectMsg>::SharedPtr detect_sub;

    // 扇页追踪
    std::shared_ptr<Track> track;
    // 滤波器
    std::shared_ptr<Kalman> pf;
    // 全部参数优化器
    std::shared_ptr<AllParamSolver> all_solver;
    // theta角优化器
    std::shared_ptr<ThetaSolver> theta_solver;
    // pnp规划器
    std::shared_ptr<PnP> pnp;

    // 最大的mse
    double max_mse = 0.1;
    // 当前mse
    double cur_mse = 10;
    // a w theta
    double *param;
    // 当前弹速
    double cur_bullet_speed;

    // 预测点在最新扇页中的偏移
    Eigen::Vector3d predict2fan_v;
    // 预测点在相机坐标中的偏移
    Eigen::Vector3d predict2camera_v;
    // 预测点在世界坐标中的偏移
    Eigen::Vector3d predict2world_v;
  };
}


# endif