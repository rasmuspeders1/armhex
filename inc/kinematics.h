/*
 * kinematics.h
 *
 *  Created on: Jul 14, 2012
 *      Author: rasmus
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <string>

/**
 * Link class
 * represents a Denavit Hartenberg Style kinematic link.
 */
class Link
{
  public:
    Link(float thetaIn, float alphaIn, float aIn, float dIn, std::string &linkNameIn);
    virtual ~Link();
    bool setTheta(float theta);
  private:
    float theta;
    float alpha;
    float a;
    float d;
    std::string linkName;
};

class Limb
{
  public:
    Limb();
    virtual ~Limb();
  private:

};

class Body
{
  public:
    Body();
    virtual ~Body();
  private:

};

#endif /* KINEMATICS_H_ */
