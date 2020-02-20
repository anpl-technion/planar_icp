/**
 * @file csm_interface.cpp
 *
 * @date Apr 4, 2013
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#include <iostream>
#include <csm_interface.h>

using namespace gtsam;
using namespace std;

/* *************************************************************** */
laser_data* convertCSM(const vector<Point2>& xy) {
  size_t nrays = xy.size();
  laser_data* ld = ld_alloc_new(nrays);

  double fov = M_PI;

  ld->min_theta = -fov / 2;
  ld->max_theta = +fov / 2;

  for (size_t i = 0; i < nrays; i++) {
    const Point2& p = xy[i];
    ld->valid[i] = true;
    ld->readings[i] = p.norm();
    ld->theta[i] = ld->min_theta + i * fov / (ld->nrays - 1);
  }
  return ld;
}

/* *************************************************************** */
sm_params defaultParametersCSM() {
  sm_params p;
  p.max_iterations = 1000;
  p.use_point_to_line_distance = 1;
  p.use_corr_tricks = 1;
  p.max_angular_correction_deg = 90.0;
  p.max_linear_correction = 2.0;
  p.epsilon_xy = 0.0001;
  p.epsilon_theta = 0.0001;
  p.max_correspondence_dist = 2.0;
  p.sigma = 0.01;
  p.restart = 1;
  p.restart_threshold_mean_error = 0.01;
  p.restart_dt = 0.01;
  p.restart_dtheta = 1.5 * M_PI / 180;
  p.clustering_threshold = 0.05;
  p.orientation_neighbourhood = 3;
  p.do_alpha_test = 0;
  p.do_alpha_test_thresholdDeg = 20.0;
  p.outliers_maxPerc = 0.95;
  p.outliers_adaptive_order = 0.7;
  p.outliers_adaptive_mult = 2.0;
  p.do_visibility_test = 0;
  p.outliers_remove_doubles = 1;
  p.do_compute_covariance = 0;
  p.debug_verify_tricks = 0;
  p.gpm_theta_bin_size_deg = 5.0;
  p.gpm_extend_range_deg = 15.0;
  p.gpm_interval = 1;
  p.laser[0] = 0.0;
  p.laser[1] = 0.0;
  p.laser[2] = 0.0;
  p.min_reading = 0.0;
  p.max_reading = 1000.0;
  p.use_ml_weights = 0;
  p.use_sigma_weights = 0;
  return p;
}

/* *************************************************************** */
static char tmp_buf[1024];
const char* friendly_pose(const double*pose) {
  sprintf(tmp_buf, "(%4.2f mm, %4.2f mm, %4.4f deg)", 1000 * pose[0],
      1000 * pose[1], pose[2] * 180 / M_PI);
  return tmp_buf;
}
/* *************************************************************** */
void printLaserDataCSM(LDP ld) {
  std::cout << "nr rays: " << ld->nrays << endl;

  cout << "min_theta: " << ld->min_theta << endl;
  cout << "max_theta: " << ld->max_theta << endl;

  cout << "true_pose: " << friendly_pose(ld->true_pose) << endl;
  cout << "odometry: " << friendly_pose(ld->odometry) << endl;
  cout << "estimate: " << friendly_pose(ld->estimate) << endl;

  for (int i = 0; i < 10; i++)
    cout << ld->valid[i] << " ";
  cout << endl;
  for (int i = 0; i < 10; i++)
    cout << ld->readings[i] << " ";
  cout << endl;
  for (int i = 0; i < 10; i++)
    cout << ld->theta[i] << " ";
  cout << endl;
}

/* *************************************************************** */
void printParametersCSM(const sm_params& p) {

  /** First scan ("ref"erence scan) */
  printLaserDataCSM(p.laser_ref);
  /** Second scan ("sens"or scan) */
  printLaserDataCSM(p.laser_sens);

  /** Where to start */
  cout << "Printing the current parameters: " << endl;
  cout << "first_guess: " << friendly_pose(p.first_guess) << endl;
  cout << "max iterations: " << p.max_iterations << endl;
  cout << "use point to line distance: " << p.use_point_to_line_distance
      << endl;
  cout << "use_corr_tricks: " << p.use_corr_tricks << endl;
  cout << "max angular correction (degrees): " << p.max_angular_correction_deg
      << endl;
  cout << "max linear correction: " << p.max_linear_correction << endl;
  cout << "epsilon xy: " << p.epsilon_xy << endl;
  cout << "epsilon theta: " << p.epsilon_theta << endl;
  cout << "max correspondence distance: " << p.max_correspondence_dist << endl;
  cout << "sigma: " << p.sigma << endl;
  cout << "restart: " << p.restart << endl;
  cout << "restart threshold mean error: " << p.restart_threshold_mean_error
      << endl;
  cout << "restart dt: " << p.restart_dt << endl;
  cout << "restart dtheta: " << p.restart_dtheta << endl;
  cout << "clustering threshold: " << p.clustering_threshold << endl;
  cout << "orientation neighborhood: " << p.orientation_neighbourhood << endl;
  cout << "do alpha test: " << p.do_alpha_test << endl;
  cout << "do alpha test threshold (degrees): " << p.do_alpha_test_thresholdDeg
      << endl;
  cout << "ouliers max Percentage: " << p.outliers_maxPerc << endl;
  cout << "ouliers adaptive order: " << p.outliers_adaptive_order << endl;
  cout << "ouliers adaptive mult: " << p.outliers_adaptive_mult << endl;
  cout << "do visibility test: " << p.do_visibility_test << endl;
  cout << "outliers remove doubles: " << p.outliers_remove_doubles << endl;
  cout << "do compute covariance: " << p.do_compute_covariance << endl;
  cout << "debug verify tricks: " << p.debug_verify_tricks << endl;
  cout << "gpm theta bin size (degrees): " << p.gpm_theta_bin_size_deg << endl;
  cout << "gpm extend range (degrees) : " << p.gpm_extend_range_deg << endl;
  cout << "gpm interval: " << p.gpm_interval << endl;
  cout << "laser: " << friendly_pose(p.laser) << endl;
  cout << "min reading: " << p.min_reading << endl;
  cout << "max reading: " << p.max_reading << endl;
  cout << "use ml weights: " << p.use_ml_weights << endl;
  cout << "use sigma weights: " << p.use_sigma_weights << endl;
}



