/*
 * main.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: rasmus
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(void) {
	float radians = 1.1;
	float result = 0;
	for(int i = 100000000; i>0; --i)
	{
		result = radians + cos(radians);
		radians = result;

	}
        printf("%f",result);
	return 0;
}
