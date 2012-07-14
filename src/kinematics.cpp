/*
 * kinematics.c++
 *
 *  Created on: Jul 14, 2012
 *      Author: rasmus
 */

#include "kinematics.h"

Link::Link(float thetaIn, float alphaIn, float aIn, float dIn, std::string &linkNameIn)
:
theta(thetaIn),
alpha(alphaIn),
a(aIn),
d(dIn),
linkName(linkNameIn)
{

}

Link::~Link()
{

}
