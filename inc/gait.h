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
     * Takes a direction input to be used for the gait calculation
     */
    bool SetDirection(kinematics::TranslationMatrix direction_matrix);

  private:
    kinematics::MatrixValue_t body_x_;
    kinematics::MatrixValue_t body_y_;
    kinematics::MatrixValue_t body_z_;

    kinematics::MatrixValue_t body_roll_;
    kinematics::MatrixValue_t body_pitch_;
    kinematics::MatrixValue_t body_yaw_;

    kinematics::TranslationMatrix lr_limb_pos_;
    kinematics::TranslationMatrix lm_limb_pos_;
    kinematics::TranslationMatrix lf_limb_pos_;

    kinematics::TranslationMatrix rr_limb_pos_;
    kinematics::TranslationMatrix rm_limb_pos_;
    kinematics::TranslationMatrix rf_limb_pos_;




};


#endif /* GAIT_H_ */
