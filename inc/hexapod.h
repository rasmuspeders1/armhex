/*
 * Hexapod.h
 *
 *  Created on: Jul 10, 2012
 *      Author: rasmus
 */

#ifndef HEXAPOD_H_
#define HEXAPOD_H_

#include "maestro_controller.h"
#include "kinematics.h"
#include "js_input.h"

class Hexapod : public MaestroController
{
  public:
    Hexapod();
    virtual ~Hexapod();

  private:
    virtual bool Update();

    //Member variable holding the positions of all servos in radians
    //The indices are the servo addresses on the Maestro controller.
    std::vector<double> servo_angles_;

    std::vector<double> get_all_leg_servo_angles();

    bool check_maestro_errors();

    /**
     * Read any input from joystick and take appropriate action.
     */
    void UpdateInput();

    JSInput js_input_;

    kinematics::Body body_;

    kinematics::Link lr_body_link_;
    kinematics::Link lr_coxa_link_;
    kinematics::Link lr_femur_link_;
    kinematics::Link lr_tibia_link_;

    kinematics::Link lm_body_link_;
    kinematics::Link lm_coxa_link_;
    kinematics::Link lm_femur_link_;
    kinematics::Link lm_tibia_link_;

    kinematics::Link lf_body_link_;
    kinematics::Link lf_coxa_link_;
    kinematics::Link lf_femur_link_;
    kinematics::Link lf_tibia_link_;

    kinematics::Link rr_body_link_;
    kinematics::Link rr_coxa_link_;
    kinematics::Link rr_femur_link_;
    kinematics::Link rr_tibia_link_;

    kinematics::Link rm_body_link_;
    kinematics::Link rm_coxa_link_;
    kinematics::Link rm_femur_link_;
    kinematics::Link rm_tibia_link_;

    kinematics::Link rf_body_link_;
    kinematics::Link rf_coxa_link_;
    kinematics::Link rf_femur_link_;
    kinematics::Link rf_tibia_link_;

    kinematics::Limb lr_leg_;
    kinematics::Limb lm_leg_;
    kinematics::Limb lf_leg_;

    kinematics::Limb rr_leg_;
    kinematics::Limb rm_leg_;
    kinematics::Limb rf_leg_;

};

#endif /* HEXAPOD_H_ */
