#ifndef okami_h
#define okami_h

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

double  get_angle(float lat1, float lon1, float lat2, float lon2);
double  to_radians(double degrees);
double to_degrees(double radians);
double  longitude(char *string);
double  conv_coords(float in_coords);
double latitude(char *string);
#endif
