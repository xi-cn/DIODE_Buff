# include "buff_predict/solver.h"

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
    return 10;
  }

  double ThetaSolver::fit(double *param, const std::list<Fan>& fans, double a, double w)
  {
    // 构建问题  
    ceres::Problem problem; 
    // 添加范围限制
    problem.SetParameterLowerBound(param, 2, -M_PI);
    problem.SetParameterUpperBound(param, 2, M_PI);

    // 加入观测点
    for (auto iter = fans.begin(); iter != fans.end(); ++iter) {
      // 无效扇页
      if (iter->filter_speed == -1) {
        continue;
      } 
      ceres::CostFunction* cost_function =  
        new ceres::AutoDiffCostFunction<TheraModel, 1, 1>(  
          new TheraModel(iter->second, iter->filter_speed, a, w));  
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0), param);  
    }  

    // 求解问题
    ceres::Solve(options, &problem, &summary); 

    // 计算mse
    return 0;
  }

}