/*
 * gait.h
 *
 *  Created on: Feb 1, 2013
 *      Author: rasmus
 */

#ifndef GAIT_H_
#define GAIT_H_

#include "kinematics.h"

/**
 * Class to hold information of a given pose of the hexapod.
 * This means body position and rotation and limb end point positions.
 */
class Pose
{
 public:
  Pose();
  virtual ~Pose();

  kinematics::TranslationMatrix GetBodyPos();
  void SetBodyPos(const kinematics::TranslationMatrix body_pos);

  void Print();

  kinematics::MatrixValue_t body_x_;
  kinematics::MatrixValue_t body_y_;
  kinematics::MatrixValue_t body_z_;

  kinematics::MatrixValue_t body_roll_;
  kinematics::MatrixValue_t body_pitch_;
  kinematics::MatrixValue_t body_yaw_;

  kinematics::TranslationMatrix lf_limb_pos_;
  kinematics::TranslationMatrix lm_limb_pos_;
  kinematics::TranslationMatrix lr_limb_pos_;
  kinematics::TranslationMatrix rf_limb_pos_;
  kinematics::TranslationMatrix rm_limb_pos_;
  kinematics::TranslationMatrix rr_limb_pos_;

};

/**
 * Base hexapod gait class
 * This implements the interface towards the main hexapod class and kinematics classes
 * It also implements the first basic gate.
 * The intention is that this class should be able to function as a base class for different gaits.
 */
class Gait
{
 public:
  Gait();
  virtual ~Gait();

  /**
   * Sets the gait direction
   * x,y indicates the translation direction and the yaw indicates the turning
   * norm of x,y vector determines step length
   */
  void direction(kinematics::MatrixValue_t x, kinematics::MatrixValue_t y, kinematics::MatrixValue_t yaw);

   /**
    * Sets the body translation relative to the limb polygon center
    */
  void body_relative_position(kinematics::MatrixValue_t x, kinematics::MatrixValue_t y, kinematics::MatrixValue_t z);
  void body_relative_rotation(kinematics::MatrixValue_t roll, kinematics::MatrixValue_t pitch, kinematics::MatrixValue_t yaw);

  /**
   * Returns the current gait target pose
   * This will change as the gait cycles along and the steps are executed as well as when the input is changed.
   */
  Pose target();

 private:

  //Step lift height
  kinematics::MatrixValue_t step_lift_;

  //Current positions for all limb end points.
  Pose current_pose_;

  //Initial position for all limb end points. Beginning of step.
  Pose start_pose_;

  //Destination position for all limb end points. End of step.
  Pose dest_pose_;
  typedef   std::vector< std::vector<bool> > pattern_t;
  typedef   std::vector< std::vector<bool> >::iterator patternIter_t;

  kinematics::TranslationMatrix direction_vector_;
  kinematics::TranslationMatrix normalized_direction_vector_;


  pattern_t pattern_;
  size_t current_gait_cycle_step_;
  int gait_cycle_direction_;
  int gait_cycle_start_;

  kinematics::MatrixValue_t body_relative_x_;
  kinematics::MatrixValue_t body_relative_y_;
  kinematics::MatrixValue_t body_relative_z_;

  kinematics::MatrixValue_t body_relative_roll_;
  kinematics::MatrixValue_t body_relative_pitch_;
  kinematics::MatrixValue_t body_relative_yaw_;


  /**
   * Method that centers the body in an appropriate position between all leg end points.
   * Basically this is to allow the body to follow the legs when they move.
   */
  void center_body();
  /**
   * get_nex
   */
  kinematics::TranslationMatrix get_next_pos(kinematics::TranslationMatrix &start, const kinematics::TranslationMatrix &end, const kinematics::TranslationMatrix &current);


};

#endif /* GAIT_H_ */
