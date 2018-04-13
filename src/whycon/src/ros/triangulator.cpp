#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <whycon/PointArray.h>
#include "triangulator.h"

whycon::Triangulator::Triangulator(ros::NodeHandle& n)
{
  points_left_sub.subscribe(n, "/whycon_left/points", 10);
  points_right_sub.subscribe(n, "/whycon_right/points", 10);

  poses_left_sub.subscribe(n, "/whycon_left/poses", 10);
  poses_right_sub.subscribe(n, "/whycon_right/poses", 10);

  /*typedef message_filters::sync_policies::ApproximateTime<whycon::PointArray, whycon::PointArray> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), points_left_sub, points_right_sub);*/
  sync = new message_filters::TimeSynchronizer<whycon::PointArray, whycon::PointArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray>
    (points_left_sub, points_right_sub, poses_left_sub, poses_right_sub, 20);
    
  sync->registerCallback(boost::bind(&Triangulator::on_points, this, _1, _2, _3, _4));

  viz_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  std::string camera_left_name, camera_right_name;
  if (!n.getParam("camera_left_name", camera_left_name)) throw std::runtime_error("Private parameter \"camera_left_name\" not provided");
  if (!n.getParam("camera_right_name", camera_right_name)) throw std::runtime_error("Private parameter \"camera_right_name\" not provided");
  
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_mgr;
  sensor_msgs::CameraInfo camera_info;
  
  cam_mgr = boost::make_shared<camera_info_manager::CameraInfoManager>(n, camera_left_name);
  camera_info = cam_mgr->getCameraInfo();
  dist_coeffs_left = cv::Mat(camera_info.D.size(), 1, CV_64FC1, &camera_info.D[0]).clone();
  K_left = cv::Mat(3, 3, CV_64FC1, &camera_info.K[0]).clone();
  R_left = cv::Mat(3, 3, CV_64FC1, &camera_info.R[0]).clone();
  P_left = cv::Mat(3, 4, CV_64FC1, &camera_info.P[0]).clone();
  ROS_INFO_STREAM("dist coeffs left " << dist_coeffs_left);
  ROS_INFO_STREAM("K/P/R left " << K_left << " " << P_left << " " << R_left);
  
  cam_mgr = boost::make_shared<camera_info_manager::CameraInfoManager>(n, camera_right_name);
  camera_info = cam_mgr->getCameraInfo();
  dist_coeffs_right = cv::Mat(camera_info.D.size(), 1, CV_64FC1, &camera_info.D[0]).clone();
  K_right = cv::Mat(3, 3, CV_64FC1, &camera_info.K[0]).clone();
  R_right = cv::Mat(3, 3, CV_64FC1, &camera_info.R[0]).clone();
  P_right = cv::Mat(3, 4, CV_64FC1, &camera_info.P[0]).clone();
  ROS_INFO_STREAM("dist coeffs right " << dist_coeffs_right);
  ROS_INFO_STREAM("K/P/R right " << K_right << " " << P_right << " " << R_right);

  //R = (cv::Mat_<double>(3, 3) << 0.9998201107700715, 0.01481867853984462, 0.011838617573635324, -0.01492480437321419, 0.9998487689673721, 0.008926892452136617, -0.011704542457666821, -0.009101975651663522, 0.9998900728205542);
}

whycon::Triangulator::~Triangulator(void)
{
  delete sync;
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
cv::Mat_<float> LinearLSTriangulation(cv::Point3f u,       //homogenous image point (u,v,1)
                   cv::Matx34f P,       //camera 1 matrix
                   cv::Point3f u1,      //homogenous image point in 2nd camera
                   cv::Matx34f P1       //camera 2 matrix
                                   )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Matx43f A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
          u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
          u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
          u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    cv::Mat_<float> B = (cv::Mat_<float>(4,1) <<    -(u.x*P(2,3)    -P(0,3)),
                      -(u.y*P(2,3)  -P(1,3)),
                      -(u1.x*P1(2,3)    -P1(0,3)),
                      -(u1.y*P1(2,3)    -P1(1,3)));
 
    cv::Mat_<float> X;
    cv::solve(A,B,X,cv::DECOMP_SVD);
    return X;
}

#define EPSILON 0.001

cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,    //homogenous image point (u,v,1)
                                              cv::Matx34d P,          //camera 1 matrix
                                              cv::Point3d u1,         //homogenous image point in 2nd camera
                                              cv::Matx34d P1          //camera 2 matrix
                                            ) {
    float wi = 1, wi1 = 1;
    cv::Mat_<double> X(4,1);
    cv::Mat_<double> X_;

    /*cv::Mat_<float> X_ = LinearLSTriangulation(u,P,u1,P1);
    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;*/

    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        //reweight equations and solve
        cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        cv::solve(A,B,X_,cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0f;
        //ROS_INFO_STREAM("A " << A << " B "<< B << " X_ " << X_);
         
        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);
         
        //breaking point
        double e1 = fabsf(wi - p2x);
        double e2 = fabsf(wi1 - p2x1);
        //ROS_INFO_STREAM("Xin " << X << " " << P << " " << P1);
        //ROS_INFO_STREAM("e " << e1 << " " << e2  << " " << p2x << " " << p2x1);
        ROS_INFO_STREAM("X " << X << " e " << e1 << " " << e2); 
        if(e1 <= EPSILON && e2 <= EPSILON) { break; }
         
        wi = p2x;
        wi1 = p2x1;
    }
    return X;
}

void triangulatePoints2(const cv::Mat& P1, const cv::Mat& P2, const std::vector<cv::Point2d>& points1, const std::vector<cv::Point2d>& points2, cv::Mat& out)
{
  for (size_t i = 0; i < points1.size(); i++) {
    cv::Vec3d point1(points1[i].x, points1[i].y, 1);
    cv::Vec3d point2(points2[i].x, points2[i].y, 1);
    out.row(i) = IterativeLinearLSTriangulation(point1, P1, point2, P2).t();
  }
}

void whycon::Triangulator::on_points(const whycon::PointArray::ConstPtr& points_left, const whycon::PointArray::ConstPtr& points_right,
                                    const geometry_msgs::PoseArray::ConstPtr& poses_left, const geometry_msgs::PoseArray::ConstPtr& poses_right)
{
  if (points_left->points.size() != points_right->points.size())
    ROS_WARN_STREAM("skipped since not equal number of points");

  ROS_INFO_STREAM("seq " << points_left->header.stamp << " " << points_right->header.stamp);

  size_t points_n = points_left->points.size();
  std::vector<cv::Point2d> points_left_vec, points_right_vec;
  points_left_vec.reserve(points_n);
  points_right_vec.reserve(points_n);

  for (size_t i = 0; i < points_n; i++) {
    points_left_vec.push_back(cv::Point2f(points_left->points[i].x, points_left->points[i].y));
    points_right_vec.push_back(cv::Point2f(points_right->points[i].x, points_right->points[i].y));
  }

  std::vector<cv::Point2d> points_left_vec_u, points_right_vec_u;

  cv::Mat realR, realT, realK;
  cv::decomposeProjectionMatrix(P_right, realK, realR, realT);
  ROS_INFO_STREAM("rot: " << realR << " " << realT << " tx " << (realT.at<double>(0) / realT.at<double>(3)));

  cv::Mat P1, P2;
  /*P_left.convertTo(P1, CV_32FC1);
  P_right.convertTo(P2, CV_32FC1);  */
  P1 = cv::Mat::eye(3, 4, CV_64FC1);
  P2 = cv::Mat::eye(3, 4, CV_64FC1);
  P2.at<double>(0, 3) = P_right.at<double>(0, 3) / P_right.at<double>(0, 0);
  //P2.at<float>(0, 3) = -(realT.at<float>(0) / realT.at<float>(3));

  ROS_INFO_STREAM("P1 " << P1);
  ROS_INFO_STREAM("P2 " << P2);

  cv::undistortPoints(points_left_vec, points_left_vec_u, K_left, dist_coeffs_left, R_left/*, P1*/);
  cv::undistortPoints(points_right_vec, points_right_vec_u, K_right, dist_coeffs_right, R_right/*, P2*/);

  for (size_t i = 0; i < points_n; i++) {
    ROS_INFO_STREAM("pixl " << points_left_vec[i].x << " " << points_left_vec[i].y);
    ROS_INFO_STREAM("pixr " << points_right_vec[i].x << " " << points_right_vec[i].y);
    ROS_INFO_STREAM("l " << points_left_vec_u[i].x << " " << points_left_vec_u[i].y);
    ROS_INFO_STREAM("r " << points_right_vec_u[i].x << " " << points_right_vec_u[i].y);
    ROS_INFO_STREAM("lp " << poses_left->poses[i].position.x << " " << poses_left->poses[i].position.y << " " << poses_left->poses[i].position.z);
    ROS_INFO_STREAM("rp " << poses_right->poses[i].position.x << " " << poses_right->poses[i].position.y << " " << poses_right->poses[i].position.z);

    points_right_vec[i].y = points_left_vec[i].y;
  }

  /*cv::Mat P1 = eye:
  cv::Mat P2 = R teye*/
  cv::Mat points4d(points_n, 4, CV_64FC1);
  /*cv::Mat P1p = cv::Mat::eye(3, 4, CV_32FC1);
  cv::Mat P2p = cv::Mat::eye(3, 4, CV_32FC1);
  P2p.at<float>(0, 3) = (P2.at<float>(0, 3) - P2.at<float>(0, 2)) / P2.at<float>(0, 0);
  ROS_INFO_STREAM("P1P " << P1p << " " << P2p);*/
  //cv::triangulatePoints(P1, P2, points_left_vec_u, points_right_vec_u, points4d);
  triangulatePoints2(P1, P2, points_left_vec_u, points_right_vec_u, points4d);

  
  visualization_msgs::Marker marker;
  marker.header = points_left->header;
  marker.ns = "points";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1;
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;

  //points4d = ((realR * cv::Mat::eye(3, 4, CV_32FC1)) * points4d.t()).t();
  
  for (size_t i = 0; i < points_n; i++) {
    cv::Vec4d hom_point(points4d.at<double>(i, 0), points4d.at<double>(i, 1), points4d.at<double>(i, 2), points4d.at<double>(i, 3));
    cv::Vec3d inhom_point(hom_point(0) / hom_point(3), hom_point(1) / hom_point(3), hom_point(2) / hom_point(3));
    
    geometry_msgs::Point marker_point;
    cv::Vec3d rotated_point = cv::Matx33d(R_left) * inhom_point;
    marker_point.x = rotated_point(0);
    marker_point.y = rotated_point(1);
    marker_point.z = rotated_point(2);
    marker.points.push_back(marker_point);
    ROS_INFO_STREAM("inhom: " << inhom_point);
    ROS_INFO_STREAM("hom: " << hom_point);
    ROS_INFO_STREAM("rot point: " << rotated_point);

    /* reprojection error */
    cv::Vec3d rep1 = cv::Matx34d(P1) * hom_point;
    cv::Vec3d rep2 = cv::Matx34d(P2) * hom_point;
    rep1 = rep1 * 1/rep1(2);
    rep2 = rep2 * 1/rep2(2);
    ROS_INFO_STREAM("rep: " << rep1 << " " << rep2);
    ROS_INFO_STREAM("err: " << cv::norm(cv::Vec2d(rep1(0), rep1(1)), cv::Vec2d(points_left_vec_u[i])) << " " << cv::norm(cv::Vec2d(rep2(0), rep2(1)), cv::Vec2d(points_right_vec_u[i])));
  }

  viz_pub.publish(marker);
}


