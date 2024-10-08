# include "buff_detect/pnp.h"

namespace Buff
{
  PnP::PnP(std::string path="")
  {
    // 相机内参  
    cameraMatrix = (cv::Mat_<double>(3, 3) << 1675.042871, 0, 623.481121,
                                              0, 1674.696037, 556.002821,
                                              0, 0, 1);  
    // 畸变系数  
    distCoeffs = (cv::Mat_<double>(1, 5) << -0.06719080320827164, 0.08771589756714641, 0.0, 0.0009056377800116876, -0.004604447207374161);

    // 能量机关世界坐标
    objectPoints.push_back(cv::Point3f(-0.1125, -0.027, 0));
    objectPoints.push_back(cv::Point3f(-0.1125, 0.027, 0));
    objectPoints.push_back(cv::Point3f(0, 0.7, 0.05));
    objectPoints.push_back(cv::Point3f(0.1125, 0.027, 0));
    objectPoints.push_back(cv::Point3f(0.1125, -0.027, 0));

  }

  // pnp计算 图像 关键点
  global_msg::msg::DetectMsg PnP::run(cv::Mat &img, KeyPoints& key_points)
  {
    // 旋转和平移
    cv::Mat rvec;
    cv::Mat tvec;

    solvePnP(objectPoints, key_points.point, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
    solvePnP(objectPoints, key_points.point, cameraMatrix, distCoeffs, rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
    global_msg::msg::DetectMsg detect_msg;

    // 能量机关到相机的旋转 tf格式
    tf2::Quaternion buff2camera_tf;
    buff2camera_tf.setRotation(tf2::Vector3(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)),
            cv::norm(rvec));
    
    // 设置旋转
    detect_msg.buff2camera_q.set__x(buff2camera_tf.getX());
    detect_msg.buff2camera_q.set__y(buff2camera_tf.getY());
    detect_msg.buff2camera_q.set__z(buff2camera_tf.getZ());
    detect_msg.buff2camera_q.set__w(buff2camera_tf.getW());

    // 设置平移
    detect_msg.buff2camera_v.set__x(tvec.at<double>(0));
    detect_msg.buff2camera_v.set__y(tvec.at<double>(1));
    detect_msg.buff2camera_v.set__z(tvec.at<double>(2));

    return detect_msg;
  }

  // 将空间点映射到图像上
  cv::Point PnP::projectPoints(cv::Mat centerPoint, cv::Mat rvec, cv::Mat tvec)
  {
    cv::Mat projectPoint = cv::Mat_<double>(1, 2);
    // 将相机中心映射到图片上
    cv::projectPoints(centerPoint, rvec, tvec, cameraMatrix, distCoeffs, projectPoint);

    cv::Point p(projectPoint.at<double>(0), projectPoint.at<double>(1));
    return p;
  }
} // namespace Buff
