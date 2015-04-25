/**
 * @file csm_interface.h
 *
 * @brief Functions to convert between our types and Andrea Censi's types
 * 
 * @date Apr 4, 2013
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <csm/csm.h>
#include <gtsam/geometry/Point2.h>

/** CSM pose in mm/deg string */
const char* friendly_pose(const double* pose);

/** Convert from 2D points to Andrea's format */
laser_data* convertCSM(const std::vector<gtsam::Point2>& xy);

//** Fill in default parameters */
sm_params defaultParametersCSM();

//** print parameters */
void printParametersCSM(const sm_params& p);

