# ifndef SOLVER
# define SLOVER

# include <ceres/ceres.h>
# include "buff_predict/tracker.h"

namespace Buff
{
  struct AllParamModel {  
    AllParamModel(const double& t, const double& speed)  
      : x(t), y(speed){}  
  
    template <typename T>  
    bool operator()(const T* const param, T* residuals) const {  
      residuals[0] = T(y) - (param[0] * ceres::sin(param[1] * T(x) + T(param[2])) + 2.09 - param[0]);  
      return true;  
    }  
  
    const double x;  
    const double y;  
  };

  struct TheraModel {
    TheraModel(const double& t, const double& speed, const double a_, const double w_)
      : x(t), y(speed), a(a_), w(w_) {}
    
    template <typename T>
    bool operator()(const T* const param, T* residuals) const {
      residuals[0] = T(y) - (a * ceres::sin(w * T(x) + T(param[2])) + 2.09 - a);
      return true;
    }

    const double x;
    const double y;
    const double a;
    const double w;
  };

  class AllParamSolver
  {
  public:
    AllParamSolver();
    double fit(double *param, const std::list<Fan>& fans);
  protected:
    // 配置求解器
    ceres::Solver::Options options;  
    // 求解总结
    ceres::Solver::Summary summary; 
  };

  class ThetaSolver : public AllParamSolver
  {
  public:
    ThetaSolver() : AllParamSolver() {}
    double fit(double *param, const std::list<Fan>& fans, double a, double w);
  };

}


# endif