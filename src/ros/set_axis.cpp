#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "set_axis.h"

namespace std {
	std::ostream& operator<<(std::ostream& str, const tf::Matrix3x3& m)
	{
		for (int i = 0; i < 3; i++) {
			str << std::endl;
			for (int j = 0; j < 3; j++)
				str << m[i][j] << " ";
		}

		return str;
	}

	std::ostream& operator<<(std::ostream& str, const tf::Vector3& v)
	{
		str << v.getX() << " " << v.getY() << " " << v.getZ();

		return str;
	}
}


whycon::AxisSetter::AxisSetter(ros::NodeHandle &n)
{
	transforms_set = false;

	if (!n.getParam("xscale", xscale) || !n.getParam("yscale", yscale)) throw std::runtime_error("Please specify xscale and yscale");

	// axis should be specified in same order as detect_square
	if (n.getParam("axis_order", axis_order)) {
		if (axis_order.size() != 4) throw std::runtime_error("Exactly four indices are needed for specifying axis");
	}

	poses_sub = n.subscribe("/whycon/poses", 1, &AxisSetter::on_poses, this);
	image_sub = n.subscribe("/camera/image_rect_color", 1, &AxisSetter::on_image, this);
	image_pub = n.advertise<sensor_msgs::Image>("image", 1);
}

/**
 * Return four tf::Point's ordered as (0,0),(0,1),(1,0),(1,1)
 */
void whycon::AxisSetter::detect_square(std::vector<tf::Point>& points)
{
	tf::Point vectors[3];
	for (int i = 0; i < 3; i++) {
		vectors[i] = points[i + 1] - points[0];
	}

  int min_prod_i = 0;
  float min_prod = std::numeric_limits<float>::max();
  for (int i = 0; i < 3; i++)
	{
    float prod = fabsf(vectors[(i + 2) % 3].dot(vectors[i]));
    if (prod < min_prod) { min_prod = prod; min_prod_i = i; }
  }

  int axis1_i = (((min_prod_i + 2) % 3) + 1);
  int axis2_i = (min_prod_i + 1);
  if (fabsf(points[axis1_i].getX()) < fabsf(points[axis2_i].getX())) std::swap(axis1_i, axis2_i);
  int xy_i = 0;
  for (int i = 1; i <= 3; i++) if (i != axis1_i && i != axis2_i) { xy_i = i; break; }

	std::vector<tf::Point> points_original(points);
	points.resize(4);
	points[0] = points_original[0];
	points[1] = points_original[axis1_i];
	points[2] = points_original[axis2_i];
	points[3] = points_original[xy_i];

	ROS_INFO_STREAM("Axis: (0,0) -> 0, (1,0) -> " << axis1_i << ", (0,1) -> " << axis2_i << ", (1,1) -> " << xy_i);
}

void whycon::AxisSetter::build_square(std::vector<tf::Point>& points)
{
	std::vector<tf::Point> points_original(points);
	points.resize(4);
	for (int i = 0; i < 4; i++)
		points[i] = points_original[axis_order[i]];

	ROS_INFO_STREAM("Axis: (0,0) -> " << axis_order[0] << ", (1,0) -> " << axis_order[1] << ", (0,1) -> " << axis_order[2] << ", (1,1) -> " << axis_order[3]);
}

tf::Matrix3x3 whycon::AxisSetter::compute_projection(const std::vector<tf::Point>& points, float xscale, float yscale)
{
	/* TODO: use only ROS/Eigen for this */
  std::vector<cv::Vec2d> src(4);
  for (int i = 0; i < 4; i++) src[i] = cv::Vec2d(points[i].getX(), points[i].getY()) / points[i].getZ();
	std::vector<cv::Vec2d> dest = { cv::Vec2d(0,0), cv::Vec2d(xscale, 0), cv::Vec2d(0, yscale), cv::Vec2d(xscale, yscale) };

	cv::Matx33d projection = cv::findHomography(src, dest, CV_LMEDS);

	tf::Matrix3x3 m;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m[i][j] = projection(i, j);

	return m;
}

tf::Transform whycon::AxisSetter::compute_similarity(const std::vector<tf::Point>& points)
{

	tf::Point vectors[3];
	vectors[0] = points[1] - points[0];
	vectors[1] = points[2] - points[0];
	vectors[2] = vectors[0].cross(vectors[1]);

	for (int i = 0; i < 3; i++) vectors[i].normalize();

	tf::Matrix3x3 rotation;
	for (int i = 0; i < 3; i++)
		for (int j = 0;	j < 3; j++)
			rotation[j][i] = vectors[i][j];

	tf::Quaternion Q;
	rotation.getRotation(Q);

	tf::Transform rot_transform;
	rot_transform.setRotation(Q);

	tf::Transform similarity;
	similarity.setOrigin(rot_transform.inverse() * -points[0]);
	similarity.setRotation(Q.inverse());

	ROS_INFO_STREAM("transformed origin -> " << similarity * points[0]);
	ROS_INFO_STREAM("transformed circle along X -> " << similarity * points[1]);
	ROS_INFO_STREAM("transformed circle along Y -> " << similarity * points[2]);
	ROS_INFO_STREAM("transformed circle along XY -> " << similarity * points[3]);

	/*float original_x = (points[1] - points[0]).length();
	float original_y = (points[2] - points[0]).length();
	float x = (similarity * points[1] - similarity * points[0]).length();
	float y = (similarity * points[2] - similarity * points[0]).length();
	float scaling_x = xscale / x;
	float scaling_y = yscale / y;

	float xy = (scaling_x * (similarity * points[3]) - scaling_y * (similarity * points[0])).length();
	ROS_INFO_STREAM("original X " << original_x << " original Y " << original_y << " norm X " << x << " norm Y " << y);
	ROS_INFO_STREAM("obtained XY scale: " << xy << " expected: " << sqrt(xscale * xscale + yscale * yscale) << " scaling: " << scaling_x << " " << scaling_y);*/

	return similarity;
}

void whycon::AxisSetter::write_projection(YAML::Emitter& yaml, const tf::Matrix3x3& projection)
{
	yaml << YAML::Key << "projection";
	yaml << YAML::Value << YAML::BeginSeq;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			yaml << projection[i][j];

	yaml << YAML::EndSeq;
}

void whycon::AxisSetter::write_similarity(YAML::Emitter& yaml, const tf::Transform& similarity)
{
	yaml << YAML::Key << "similarity";
	yaml << YAML::Value << YAML::BeginMap;

	yaml << YAML::Key << "origin";
	yaml << YAML::Value;

	yaml << YAML::BeginSeq;
	yaml << similarity.getOrigin().getX() << similarity.getOrigin().getY() << similarity.getOrigin().getZ();
	yaml << YAML::EndSeq;

	yaml << YAML::Key << "rotation";
	yaml << YAML::Value;
	yaml << YAML::BeginSeq;
	yaml << similarity.getRotation().getX() << similarity.getRotation().getY() << similarity.getRotation().getZ() << similarity.getRotation().getW();
	yaml << YAML::EndSeq;
	yaml << YAML::EndMap;
}

void whycon::AxisSetter::on_poses(const geometry_msgs::PoseArrayConstPtr& poses_msg)
{
	if (transforms_set) return;

	if (poses_msg->poses.size() < 4) {
		ROS_WARN_STREAM("Not computing, only " << poses_msg->poses.size() << " targets detected, need four.");
		return;
	}

	std::vector<tf::Point> points(4);
	for (int i = 0; i < 4; i++)
		tf::pointMsgToTF(poses_msg->poses[i].position, points[i]);

	if (axis_order.empty())
		detect_square(points);
	else
		build_square(points);

	tf::Matrix3x3 projection = compute_projection(points, xscale, yscale);
	tf::Transform similarity = compute_similarity(points);

	ROS_INFO_STREAM("Computed Transformations for \"" << poses_msg->header.frame_id << "\" localizer");
	ROS_INFO_STREAM("Projection:" << projection);
	ROS_INFO_STREAM("Similarity Translation:" << similarity.getOrigin());
	ROS_INFO_STREAM("Similarity Rotation:" << similarity.getBasis());

	YAML::Emitter yaml;
	yaml << YAML::BeginMap;
	write_projection(yaml, projection);
	write_similarity(yaml, similarity);
	yaml << YAML::EndMap;

	std::string filename = poses_msg->header.frame_id + "_transforms.yml";
	std::ofstream config_file(filename);
	config_file << yaml.c_str();

	ROS_INFO_STREAM("Wrote transformations to " << filename);

	transforms_set = true;
}

void whycon::AxisSetter::on_image(const sensor_msgs::ImageConstPtr& image_msg)
{
	cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg);
}
