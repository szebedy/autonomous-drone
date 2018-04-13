#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include <whycon/localization_system.h>
using namespace std;
namespace po = boost::program_options;

bool stop = false;
void interrupt(int s) {
  stop = true;
}

bool clicked = false;
void mouse_callback(int event, int x, int y, int flags, void* param) {
  if (event == CV_EVENT_LBUTTONDOWN) { cout << "clicked window" << endl; clicked = true; }
}

bool do_tracking;
bool use_gui;
int number_of_targets;
bool load_axis;
bool custom_diameter = false;

po::variables_map process_commandline(int argc, char** argv)
{
  po::options_description options_description("WhyCon options");
  options_description.add_options()
    ("help,h", "display this help")
  ;

  po::options_description mode_options("Operating mode");
  mode_options.add_options()    
    ("set-axis,s", po::value<string>(), "perform axis detection and save results to specified XML file")
    ("track,t", po::value<int>(), "perform tracking of the specified ammount of targets")
  ;

  po::options_description input_options("Input source");
  input_options.add_options()
    ("cam,c", po::value<int>(), "use camera as input (expects id of camera)")
    ("video,v", po::value<string>(), "use video as input (expects path to video file)")
    ("img,i", po::value<string>(), "use sequence of images as input (expects pattern describing sequence). "
                                      "Use a pattern such as 'directory/%03d.png' for files named 000.png to "
                                      "999.png inside said directory")
  ;
  
  po::options_description tracking_options("Tracking options");
  tracking_options.add_options()
    ("axis,a", po::value<string>(), "use specified axis definition XML file for coordinate transformation during tracking")
    ("no-axis,n", "do not transform 3D coordinates during tracking")
    ("output,o", po::value<string>(), "name to be used for all tracking output files")
  ;

  po::options_description parameter_options("Other options");
  parameter_options.add_options()
    ("inner-diameter", po::value<float>(), "use specified inner diameter (in meters) of circles")
    ("outer-diameter", po::value<float>(), "use specified outer diameter (in meters) of circles")  
    ("mat,m", po::value<string>(), "use specified matlab (.m) calibration toolbox file for camera calibration parameters")
    ("xml,x", po::value<string>(), "use specified 'camera_calibrator' file (.xml) for camera calibration parameters")
    ("service", "run as a mavconn service, outputting pose information through bus")
    ("no-gui", "disable opening of GUI")
    ("width", po::value<int>(), "input camera resolution width")
    ("height", po::value<int>(), "input camera resolution height")
  ;

  options_description.add(mode_options).add(input_options).add(tracking_options).add(parameter_options);

  po::variables_map config_vars;
  try {
    po::store(po::parse_command_line(argc, argv, options_description), config_vars);
    if (config_vars.count("help")) { cerr << options_description << endl; exit(1); }

    po::notify(config_vars);
    
    if (config_vars.count("track")) do_tracking = true;
    else if (config_vars.count("set-axis")) do_tracking = false;
    else throw std::runtime_error("Select either tracking or axis setting mode");
    
    if (!config_vars.count("mat") && !config_vars.count("xml"))
      throw std::runtime_error("Please specify one source for calibration parameters");
      
    if (!config_vars.count("cam") && !config_vars.count("video") && !config_vars.count("img"))
      throw std::runtime_error("Please specify one input source");

    if (config_vars.count("width") != config_vars.count("height"))
      throw std::runtime_error("Please specify both width and height for camera resolution");

    if (!config_vars.count("output"))
      throw std::runtime_error("Specify prefix name for output files");

    use_gui = !config_vars.count("no-gui");

    if (do_tracking) {
      if (config_vars["track"].as<int>() < 0) throw std::runtime_error("Number of circles to track should be greater than 0");
      if (config_vars.count("no-axis")) load_axis = false;
      else {
        if (!config_vars.count("axis")) throw std::runtime_error("Axis definition file is missing, if you dont need to perform coordinate transformation use --no-axis");
        load_axis = true;
      }
      if (config_vars.count("inner-diameter") && config_vars.count("outer-diameter"))
        custom_diameter = true;
      else if (config_vars.count("inner-diameter") || config_vars.count("outer-diameter"))
        throw std::runtime_error("please specify both outer and inner diameters");
      
      number_of_targets = config_vars["track"].as<int>();
    }
    else {
      if (config_vars.count("video")) throw std::runtime_error("Video input is not supported for axis definition");
      if (!use_gui && config_vars.count("cam")) throw std::runtime_error("Camera input is not supported for axis setting when GUI is disabled");
      if (config_vars.count("service")) throw std::runtime_error("Running as service is only for tracking mode");
    }
  }
  catch(const std::runtime_error& e) {
    cerr << options_description << endl << endl;
    cerr << "ERROR: " << e.what() << endl;
    exit(1);
  }
  catch(po::error& e) { 
    cerr << options_description << endl << endl;
    cerr << endl << "ERROR: " << e.what() << endl;
    exit(1);
  }   
  return config_vars;
}


int main(int argc, char** argv)
{
  signal(SIGINT, interrupt);

  /* process command line */
  po::variables_map config_vars = process_commandline(argc, argv);

  /* setup input */
  bool is_camera = config_vars.count("cam");
  cv::VideoCapture capture;
  if (is_camera) {
    int cam_id = config_vars["cam"].as<int>();
    capture.open(cam_id);
    if (config_vars.count("width")) {
      capture.set(CV_CAP_PROP_FRAME_WIDTH, config_vars["width"].as<int>());
      capture.set(CV_CAP_PROP_FRAME_HEIGHT, config_vars["height"].as<int>());
    }
  }
  else {
    std::string video_name(config_vars.count("img") ? config_vars["img"].as<string>() : config_vars["video"].as<string>());
    capture.open(video_name);    
  }
  if (!capture.isOpened()) { cout << "error opening camera/video" << endl; return 1; }

  /* load calibration */
  cv::Mat K, dist_coeff;
  if (config_vars.count("xml"))
    cv::LocalizationSystem::load_opencv_calibration(config_vars["xml"].as<string>(), K, dist_coeff);
  else
    cv::LocalizationSystem::load_matlab_calibration(config_vars["mat"].as<string>(), K, dist_coeff);

  /* init system */
  
  cv::Size frame_size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT));
  cout << "frame size: " << frame_size << endl;
  float inner_diameter = (custom_diameter ? config_vars["inner-diameter"].as<float>() : WHYCON_DEFAULT_INNER_DIAMETER);
  float outer_diameter = (custom_diameter ? config_vars["outer-diameter"].as<float>() : WHYCON_DEFAULT_OUTER_DIAMETER);
  cv::LocalizationSystem system(number_of_targets, frame_size.width, frame_size.height, K, dist_coeff,
                                outer_diameter, inner_diameter);
  cout << "using diameters (outer/inner): " << outer_diameter << " " << inner_diameter << endl;

  #ifdef ENABLE_MAVCONN
  bool run_service = config_vars.count("service");
  cv::LocalizationService service(system);
  if (run_service) service.start();
  #endif

  /* setup gui */
  if (use_gui) {
    cvStartWindowThread();
    cv::namedWindow("output", CV_WINDOW_NORMAL);
    cv::setMouseCallback("output", mouse_callback);
  }

  /* set tracking output */
  std::string output_name = config_vars["output"].as<string>();
  cv::VideoWriter video_writer;
  ofstream data_file;
  if (do_tracking) {
    video_writer.open(output_name + ".avi", CV_FOURCC('M','J','P','G'), 15, frame_size);
    if (!video_writer.isOpened()) throw std::runtime_error("error opening output video");
    data_file.open((output_name + ".log").c_str(), ios_base::out | ios_base::trunc);
    if (!data_file) throw std::runtime_error(string("error opening '") + output_name + ".log' output data file");
  }
  
  /* setup gui and start capturing / processing */
  bool is_tracking = false;
  if (!is_camera) clicked = true; // when not using camera, emulate user click so that tracking starts immediately
  cv::Mat original_frame, frame;
  long frame_idx = 0;

  /* read axis from file when in tracking mode */
  if (do_tracking) {
    if (load_axis) {
      cout << "loading axis definition file" << endl;
      system.read_axis(config_vars["axis"].as<string>());
    }
    else cout << "coordinate transform disabled" << endl;
  }

  int max_attempts = is_camera ? 1 : 5;
  int refine_steps = is_camera ? 1 : 15;

  while (!stop)
  {
    if (!capture.read(original_frame)) { cout << "no more frames left to read" << endl; break; }

    #if defined(ENABLE_FULL_UNDISTORT)
    cv::Mat undistorted;
    cv::undistort(original_frame, undistorted, K, dist_coeff, K);
    undistorted.copyTo(original_frame);
    #endif
    
    original_frame.copyTo(frame);

    if (!do_tracking) {
      if (!is_camera || clicked) {
        bool axis_was_set = system.set_axis(original_frame, max_attempts, refine_steps, config_vars["set-axis"].as<string>());
        if (!axis_was_set) throw std::runtime_error("Error setting axis!");      
        system.draw_axis(frame);
        cv::imwrite(output_name + "_axis_detected.png", frame);
        stop = true;
      }
      if (use_gui) cv::imshow("output", frame);
    }
    else {
      if (!use_gui || !is_camera || clicked) {
        if (!is_tracking) cout << "resetting targets" << endl;

        int64_t ticks = cv::getTickCount();
        is_tracking = system.localize(original_frame, !is_tracking, max_attempts, refine_steps);
        double delta_ticks = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
        
        cout << "localized all? " << is_tracking << " t: " << delta_ticks << " " << " fps: " << 1/delta_ticks << endl;
        
        for (int i = 0; i < number_of_targets; i++) {
          const cv::CircleDetector::Circle& circle = system.get_circle(i);
          if (!circle.valid) continue;
          cv::Vec3f coord = system.get_pose(circle).pos;
          cv::Vec3f coord_trans = coord;
          if (load_axis) {
            coord_trans = system.get_transformed_pose(circle).pos;
          }

          if (use_gui) {
            ostringstream ostr;
            ostr << fixed << setprecision(2);
            ostr << coord_trans << " " << i;
            circle.draw(frame, ostr.str(), cv::Vec3b(255,255,0));
          }

          data_file << setprecision(15) << "frame " << frame_idx << " circle " << i
            << " transformed: " << coord_trans(0) << " " << coord_trans(1) << " " << coord_trans(2)
            << " original: " << coord(0) << " " << coord(1) << " " << coord(2) << endl;

          #ifdef ENABLE_MAVCONN
          if (run_service) service.publish();
          #endif
        }

        video_writer << frame;
      }
      if (use_gui) cv::imshow("output", frame);
    }

    frame_idx++;
  }

  /*#ifdef ENABLE_VIEWER
  if (!stop) viewer.wait();
  #endif*/
  return 0;
}

