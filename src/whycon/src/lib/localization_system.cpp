#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <limits>
#include <whycon/circle_detector.h>
#include <whycon/localization_system.h>
using std::cout;
using std::endl;
using std::numeric_limits;

whycon::LocalizationSystem::LocalizationSystem(int _targets, int _width, int _height, const cv::Mat& _K, const cv::Mat& _dist_coeff,
																							 const whycon::DetectorParameters& parameters) :
  detector(_targets, _width, _height, parameters),
  targets(_targets), width(_width), height(_height), circle_diameter(parameters.outer_diameter)
{
  _K.copyTo(K);
  _dist_coeff.copyTo(dist_coeff);
  
  fc[0] = K.at<double>(0,0);
  fc[1] = K.at<double>(1,1);
  cc[0] = K.at<double>(0,2);
  cc[1] = K.at<double>(1,2);
  
  kc[0] = 1;
  for (int i = 0; i < 5; i++) kc[i + 1] = dist_coeff.at<double>(i);

  precompute_undistort_map();

  cout.precision(30);
}

bool whycon::LocalizationSystem::localize(const cv::Mat& image, bool reset, int attempts, int max_refine) {
  return detector.detect(image, reset, attempts, max_refine);
}

whycon::LocalizationSystem::Pose whycon::LocalizationSystem::get_pose(const whycon::CircleDetector::Circle& circle) const {
  Pose result;
  double x,y,x1,x2,y1,y2,sx1,sx2,sy1,sy2,major,minor,v0,v1;
  
  //transform the center
	transform(circle.x,circle.y, x, y);
  
  //calculate the major axis 
	//endpoints in image coords
	sx1 = circle.x + circle.v0 * circle.m0 * 2;
	sx2 = circle.x - circle.v0 * circle.m0 * 2;
	sy1 = circle.y + circle.v1 * circle.m0 * 2;
	sy2 = circle.y - circle.v1 * circle.m0 * 2;

  //endpoints in camera coords 
	transform(sx1, sy1, x1, y1);
	transform(sx2, sy2, x2, y2);

  //semiaxis length 
	major = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;
  
	v0 = (x2-x1)/major/2.0;
	v1 = (y2-y1)/major/2.0;

	//calculate the minor axis 
	//endpoints in image coords
	sx1 = circle.x + circle.v1 * circle.m1 * 2;
	sx2 = circle.x - circle.v1 * circle.m1 * 2;
	sy1 = circle.y - circle.v0 * circle.m1 * 2;
	sy2 = circle.y + circle.v0 * circle.m1 * 2;
  
	//endpoints in camera coords 
	transform(sx1, sy1, x1, y1);
	transform(sx2, sy2, x2, y2);

	//semiaxis length 
	minor = sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))/2.0;

	//construct the conic
	double a,b,c,d,e,f;
	a = v0*v0/(major*major)+v1*v1/(minor*minor);
	b = v0*v1*(1/(major*major)-1/(minor*minor));
	c = v0*v0/(minor*minor)+v1*v1/(major*major);
	d = (-x*a-b*y);
	e = (-y*c-b*x);
	f = (a*x*x+c*y*y+2*b*x*y-1);
	cv::Matx33d data(a,b,d,
									 b,c,e,
									 d,e,f);

	// compute conic eigenvalues and eigenvectors
	cv::Vec3d eigenvalues;
	cv::Matx33d eigenvectors;
	cv::eigen(data, eigenvalues, eigenvectors);

	// compute ellipse parameters in real-world
	double L1 = eigenvalues(1);
	double L2 = eigenvalues(0);
	double L3 = eigenvalues(2);
	int V2 = 0;
	int V3 = 2;

	// position
	double z = circle_diameter/sqrt(-L2*L3)/2.0;
	cv::Matx13d position_mat = L3 * sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + L2 * sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
	result.pos = cv::Vec3f(position_mat(0), position_mat(1), position_mat(2));
	int S3 = (result.pos(2) * z < 0 ? -1 : 1);
	result.pos *= S3 * z;

	/*float dist = sqrt(L1 * L1 * L1) * circle_diameter * 0.5;
	std::cout << "d1 " << dist << " " << cv::norm(result.pos) << std::endl;*/


	WHYCON_DEBUG("ellipse center: " << x << "," << y << " " << " computed position: " << result.pos << " " << result.pos / result.pos(2));

	// rotation
	cv::Matx13d normal_mat = sqrt((L2 - L1) / (L2 - L3)) * eigenvectors.row(V2) + sqrt((L1 - L3) / (L2 - L3)) * eigenvectors.row(V3);
	cv::normalize(cv::Vec3f(normal_mat(0), normal_mat(1), normal_mat(2)), result.rot, 1, cv::NORM_L2SQR);
	result.rot(0) = atan2(result.rot(1), result.rot(0));
	result.rot(1) = acos(result.rot(2));
	result.rot(2) = 0; /* not recoverable */
	/* TODO: to be checked */

	/*cv::Matx33d data_inv;
	cv::invert(data, data_inv);
	cv::Matx31d projection = data_inv * normal_mat.t();
	std::cout << "det " << cv::determinant(data_inv) << std::endl;
	//cv::Vec3f new_center = cv::normalize(cv::Vec3f(projection(0), projection(1), projection(2))) * dist;
	cv::Vec3f new_center = cv::Vec3f(projection(0) / projection(2), projection(1) / projection(2), 1) * dist;
	std::cout << "center: " << new_center(0) << "," << new_center(1) << "," << new_center(2) << " vs " << result.pos << std::endl;
	std::cout << "normalized: " << cv::normalize(new_center) << " " << cv::normalize(result.pos) << std::endl;*/

  return result;
}

const whycon::CircleDetector::Circle& whycon::LocalizationSystem::get_circle(int id)
{
  return detector.circles[id];
}

whycon::LocalizationSystem::Pose whycon::LocalizationSystem::get_pose(int id) const
{
  return get_pose(detector.circles[id]);
}

/* normalize coordinates: move from image to canonical and remove distortion */
void whycon::LocalizationSystem::transform(double x_in, double y_in, double& x_out, double& y_out) const
{
  #if defined(ENABLE_FULL_UNDISTORT)
  x_out = (x_in-cc[0])/fc[0];
  y_out = (y_in-cc[1])/fc[1];
  #else
  std::vector<cv::Vec2d> src(1, cv::Vec2d(x_in, y_in));
  std::vector<cv::Vec2d> dst(1);
  cv::undistortPoints(src, dst, K, dist_coeff);
  x_out = dst[0](0); y_out = dst[0](1);
  #endif
}

void whycon::LocalizationSystem::precompute_undistort_map(void)
{
  undistort_map.create(height, width, CV_32FC2);
  for (int i = 0; i < height; i++) {
    std::vector<cv::Vec2f> coords_in(width);
    for (int j = 0; j < width; j++)
      coords_in[j] = cv::Vec2f(j,i); // TODO: reverse y? add 0.5?

    undistortPoints(coords_in, undistort_map.row(i), K, dist_coeff);
  }
}
