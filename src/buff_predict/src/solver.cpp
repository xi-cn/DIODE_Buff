# include "buff_predict/solver.h"
# include <buff_predict/matplotlibcpp.h>


namespace Buff
{
  AllParamSolver::AllParamSolver()
  {
    // 求解类型
    options.linear_solver_type = ceres::DENSE_QR;  
    // 不输出过程消息
    options.minimizer_progress_to_stdout = false;  
    // 最小梯度
    options.gradient_tolerance = 0.1;
  }

  double AllParamSolver::fit(double *param, const std::list<Fan>& fans)
  {
    // 构建问题  
    ceres::Problem problem; 

    // 避免所有点都无效
    bool is_valid = false;
    // 加入观测点
    for (auto iter = fans.begin(); iter != fans.end(); ++iter) {
      // 无效扇页
      if (iter->filter_speed == -1) {
        continue;
      }  
      is_valid = true;
      ceres::CostFunction* cost_function =  
        new ceres::AutoDiffCostFunction<AllParamModel, 1, 3>(  
          new AllParamModel(iter->second, iter->filter_speed));  
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), param);  
    }  

    if (!is_valid) {
      return 1000;
    }

    // 添加范围限制
    problem.SetParameterLowerBound(param, 0, 0.780);
    problem.SetParameterUpperBound(param, 0, 1.045);

    problem.SetParameterLowerBound(param, 1, 1.884);
    problem.SetParameterUpperBound(param, 1, 2.000);

    problem.SetParameterLowerBound(param, 2, -M_PI);
    problem.SetParameterUpperBound(param, 2, M_PI);

    // 求解问题
    ceres::Solve(options, &problem, &summary); 
    // 计算mse
    return mse(param, fans);
  }


  // 计算mse
  double AllParamSolver::mse(double *param, const std::list<Fan>& fans)
  {
    // 时间
    std::vector<double> secs;
    // 原始速度
    std::vector<double> raw_speed;
    // 拟合速度
    std::vector<double> fit_speed;

    for (auto it = fans.begin(); it != fans.end(); ++it) {
      // 无效扇页
      if (it->filter_speed == -1) {
        continue;
      }

      double second = it->second;
      double speed = it->filter_speed;
      double f_speed = param[0] * sin(param[1] * second + param[2]) + 2.090 - param[0];

      secs.push_back(second);
      raw_speed.push_back(speed);
      fit_speed.push_back(f_speed);
    }

    // 计算mse
    double mse = 0;
    if (fit_speed.size() == 0) {
      return 100;
    }

    for (int i = 0; i < fit_speed.size(); ++i) {
      mse += (fit_speed[i] - raw_speed[i]) * (fit_speed[i] - raw_speed[i]);
    }

    mse /= fit_speed.size();

    return mse;
  }

  double ThetaSolver::fit(double *param, const std::list<Fan>& fans, double a, double w)
  {
    double *theta = param + 2;

    // 构建问题  
    ceres::Problem problem; 
    // 避免所有点都无效
    bool is_valid = false;
    // 加入观测点
    for (auto iter = fans.begin(); iter != fans.end(); ++iter) {
      // 无效扇页
      if (iter->filter_speed == -1) {
        continue;
      } 
      is_valid = true;
      ceres::CostFunction* cost_function =  
        new ceres::AutoDiffCostFunction<TheraModel, 1, 1>(  
          new TheraModel(iter->second, iter->filter_speed, a, w));  
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), theta);  
    }  

    if (is_valid == false) {
      return 0;
    }

    // 添加范围限制
    problem.SetParameterLowerBound(theta, 0, -M_PI);
    problem.SetParameterUpperBound(theta, 0, M_PI);

    // 求解问题
    ceres::Solve(options, &problem, &summary); 

    // 计算mse
    return mse(param, fans);
  }

}