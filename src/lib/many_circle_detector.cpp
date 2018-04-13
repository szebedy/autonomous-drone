#include <iostream>
#include <whycon/many_circle_detector.h>
using namespace std;

whycon::ManyCircleDetector::ManyCircleDetector(int _number_of_circles, int _width, int _height, const whycon::DetectorParameters& parameters) :
  context(_width, _height), width(_width), height(_height), number_of_circles(_number_of_circles)
{
  circles.resize(number_of_circles);
  last_valid_circles.resize(number_of_circles);
  detectors.resize(number_of_circles, CircleDetector(width, height, &context, parameters));
}

whycon::ManyCircleDetector::~ManyCircleDetector(void) {
}

bool whycon::ManyCircleDetector::detect(const cv::Mat& input, bool reset, int max_attempts, int refine_max_step)
{
  bool all_detected = true;
  bool do_fast_cleanup = true;

  /* if reset was asked, looking for circles anywhere in the image */
  if (reset) { last_valid_circles.clear(); last_valid_circles.resize(number_of_circles); }

  for (int i = 0; i < number_of_circles; i++) {
    WHYCON_DEBUG("detecting circle " << i);
    
    for (int j = 0; j < max_attempts; j++) {
      WHYCON_DEBUG("attempt " << j);
      circles[i] = last_valid_circles[i]; /* start from last known valid circle's position */

      bool fast_cleanup_possible;
      for (int refine_counter = 0; refine_counter < refine_max_step; refine_counter++)
      {
        if (refine_counter > 0) WHYCON_DEBUG("refining step " << refine_counter << "/" << refine_max_step);
        int prev_threshold = detectors[i].get_threshold();

        //int64_t ticks = cv::getTickCount();

        WHYCON_DEBUG("using threshold " << detectors[i].get_threshold());
        circles[i] = detectors[i].detect(input, fast_cleanup_possible, circles[i]);
        WHYCON_DEBUG("threshold is now " << detectors[i].get_threshold());

        do_fast_cleanup = do_fast_cleanup && fast_cleanup_possible; /* if a single detector does not allow for fast cleanup, perform global cleanup */

        /*double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
        cout << "t: " << delta << " " << " fps: " << 1/delta << endl;*/

        /* if detection failed, nothing else to do here, continue with next circle */
        if (!circles[i].valid) break;

        /* if refinenment is requested and there are still refine steps to do... */
        if (refine_max_step > 1 && refine_counter < refine_max_step) {
          if (detectors[i].get_threshold() == prev_threshold) break; /* threshold converged, no more refinment possible */
        }
      }

      if (circles[i].valid) {
        WHYCON_DEBUG("detection of circle " << i << " ok");
        last_valid_circles[i] = circles[i];

        WHYCON_DEBUG("adding segment ids: " << context.total_segments - 1 << " and " << context.total_segments - 2);
        /* inser segment_ids corresponding to inner and outer parts of the valid circle detected */
        context.valid_segment_ids.insert(context.total_segments - 1);
        context.valid_segment_ids.insert(context.total_segments - 2);

        break; /* detection was successful, dont keep trying */
      }
    }

    // DEBUG
    /*cv::Mat buffer;
    context.debug_buffer(input, buffer);
    cv::imshow("bleh", buffer);
    cv::waitKey();*/

    /* detection was not possible for this circle, so no other circles will be found, thus abort search */
    if (!circles[i].valid) { all_detected = false; break; }
  }

  //int64_t ticks = cv::getTickCount();
  /* do cleanup */
  /*if (do_fast_cleanup) {
    for (int i = 0; i < number_of_circles; i++) {

    }
  }
  else*/ context.cleanup_buffer(); /* global cleanup */
  /* TODO: re-enable fast cleanup */
  //double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
  //WHYCON_DEBUG("cleanup : " << delta);

  /* reset internal context's ids */
  context.reset();

  // DEBUG
  /*cv::Mat buffer;
  context.debug_buffer(input, buffer);
  cv::imshow("bleh", buffer);
  cv::waitKey();*/
  
  return all_detected;
}
