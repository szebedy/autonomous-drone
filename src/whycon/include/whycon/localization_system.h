#ifndef __LOCALIZATION_SYSYEM__
#define __LOCALIZATION_SYSYEM__

#include <opencv2/opencv.hpp>
#include <whycon/many_circle_detector.h>

namespace whycon {
  class LocalizationSystem {
    public:
      LocalizationSystem(int targets, int width, int height, const cv::Mat& K, const cv::Mat& dist_coeff,
                         const whycon::DetectorParameters& parameters = DetectorParameters());
      
      bool localize(const cv::Mat& image, bool reset = false, int attempts = 1, int max_refine = 1);
      
      // TODO: use double?
      struct Pose {
        cv::Vec3f pos;
        cv::Vec3f rot; // pitch, roll, yaw
      };
      
      Pose get_pose(int id) const;
      Pose get_pose(const CircleDetector::Circle& circle) const;
      const CircleDetector::Circle& get_circle(int id);
      
      ManyCircleDetector detector;
      
      int targets, width, height;
      
    private:
      cv::Mat K, dist_coeff;
      float circle_diameter;
      double fc[2]; // focal length X,Y
      double cc[2]; // principal point X,Y
      double kc[6]; // distortion coefficients
      void transform(double x_in, double y_in, double& x_out, double& y_out) const;

      void precompute_undistort_map(void);
      cv::Mat undistort_map;
  };
}

#endif
