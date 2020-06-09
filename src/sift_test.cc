
#include <iostream>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Eigen
#include <eigen3/Eigen/Core>

// GFLAGS
#include <gflags/gflags.h>

// GLOG
#include <glog/logging.h>

// VLFEAT Library.
extern "C" {
#include "extern/lib/include/vl/sift.h"
}

using namespace std;

void LoadImages(const std::string& image_path, cv::Mat& img, cv::Mat& gray_img,
                cv::Mat& float_img) {
  img = cv::imread(image_path);

  cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

  gray_img.convertTo(float_img, CV_32FC1);
}

DEFINE_string(image_path, "", "Path to the image.");

// Maximum scaled dimension to extract descriptors. If the dimension is larger
// than this then we begin to have memory and speed issues.
// CONST Parameters.
DEFINE_int32(kMaxScaledDim, 3600, "");
DEFINE_int32(kNumSiftDimensions, 128, "");

// SIFT Parameters
DEFINE_int32(num_octaves, -1, "");
DEFINE_int32(num_levels, 3, "");
DEFINE_int32(def_first_octave, -1, "");

DEFINE_double(peak_threshold, 1.2f, "");
DEFINE_double(edge_threshold, 10.0f, "");
DEFINE_double(norm_threshold, 0.0f, "");
DEFINE_double(magnif, 3.0f, "");
DEFINE_double(window_size, 2.0f, "");

DEFINE_bool(root_sift, false, "");
DEFINE_bool(upright_sift, false, "");

double GetValidFirstOctave(const int first_octave, const int width, const int height) {
  const int max_dim = std::max(width, height);
  int valid_first_octave = first_octave;
  double scale_factor = std::pow(2.0, -1 * valid_first_octave);
  while (max_dim * scale_factor >= FLAGS_kMaxScaledDim) {
    scale_factor /= 2.0;
    ++valid_first_octave;
  }
  return valid_first_octave;
}

void DetectAndExtractDescriptors(cv::Mat& float_img) {
  std::vector<cv::KeyPoint> keypoints;

  // 2. Compute Valid First Octave.
  const int first_octave =
      GetValidFirstOctave(FLAGS_def_first_octave, float_img.cols, float_img.rows);

  // 3. Create new sift filter.
  VlSiftFilt* sift_filter = vl_sift_new(float_img.cols, float_img.rows, FLAGS_num_octaves,
                                        FLAGS_num_levels, first_octave);

  // 4. Set Parameters.
  vl_sift_set_peak_thresh(sift_filter, FLAGS_peak_threshold / 255.0);
  vl_sift_set_edge_thresh(sift_filter, FLAGS_edge_threshold / 255.0);
  vl_sift_set_norm_thresh(sift_filter, FLAGS_norm_threshold / 255.0);
  vl_sift_set_magnif(sift_filter, FLAGS_magnif);
  vl_sift_set_window_size(sift_filter, FLAGS_window_size);

  // 5. Process First Octave.
  int octave_no = 1;
  int vl_status =
      vl_sift_process_first_octave(sift_filter, reinterpret_cast<float*>(float_img.data));

  std::vector<Eigen::VectorXf> descriptors;

  // 6. Process Octave until we cant anymore.
  while (vl_status != VL_ERR_EOF) {
    // Detect Key Points.
    vl_sift_detect(sift_filter);

    // Get Key Points.
    const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift_filter);
    const int num_keypoints = vl_sift_get_nkeypoints(sift_filter);

    LOG(INFO) << "Processing Octave : " << octave_no;
    LOG(INFO) << "Octave Width : " << vl_sift_get_octave_width(sift_filter);
    LOG(INFO) << "Octave Height : " << vl_sift_get_octave_height(sift_filter);
    LOG(INFO) << "Num Keypoints : " << num_keypoints;

    // Calc Orientation for keypoints detected.
    for (int i = 0; i < num_keypoints; ++i) {
      // Calculate orientations of the keypoint.
      double angles[4];
      int num_angles = vl_sift_calc_keypoint_orientations(sift_filter, angles, &vl_keypoints[i]);

      // Compute for Each Key Point.
      Eigen::VectorXf descriptor(FLAGS_kNumSiftDimensions);
      for (int j = 0; j < num_angles; ++j) {
        descriptor.setZero();
        vl_sift_calc_keypoint_descriptor(sift_filter, descriptor.data(), &vl_keypoints[i],
                                         angles[j]);
        descriptors.push_back(descriptor);

        cv::KeyPoint keypoint;
        keypoint.pt.x = vl_keypoints[i].x;
        keypoint.pt.y = vl_keypoints[i].y;
        keypoint.size = vl_keypoints[i].sigma;
        keypoint.angle = angles[j];
        keypoints.push_back(keypoint);
      }
    }
    vl_status = vl_sift_process_next_octave(sift_filter);
    octave_no = octave_no + 1;
  }

  vl_sift_delete(sift_filter);
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = 1;
  FLAGS_stderrthreshold = google::GLOG_INFO;
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Vlfeat SIFT Test Started.";

  // 1. Load Images.
  cv::Mat img, gray_img, float_img;
  LoadImages(FLAGS_image_path, img, gray_img, float_img);

  // 2. Detect Key Points and Compute Descriptor.
  DetectAndExtractDescriptors(float_img);

  LOG(INFO) << "Vlfeat SIFT Test Finished.";

  return 0;
}
