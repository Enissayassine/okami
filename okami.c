#include "okami.h"

double latitude(char *string)
{
  int i;
  int j;
  char  *lat;

  i = 0;
  while (string[i] && string[i] != ',')
    i++;
  i++;
  if (!(lat = malloc(sizeof(char) * 10)))
    return (0);
  j = 0;
  while (j < 9)
    lat[j++] = string[i++];
  lat[j] = '\0';
  return (atof(lat));
}

double  conv_coords(float in_coords)
{
  //Initialize the location.
  double f = in_coords;
  // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
  // firsttowdigits should be 77 at this point.
  int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
  double nexttwodigits = f - (double)(firsttwodigits*100);
  double theFinalAnswer = (double)(firsttwodigits + nexttwodigits/60.0);
  return theFinalAnswer;
}

double  longitude(char *string)
{
  int i;
  int j;
  int vir;
  char  *lon;

  i = 0;
  vir = 0;
  while (string[i] && vir < 3)
  {
    if (string[i++] == ',')
      vir++;
  }
  if (!(lon = malloc(sizeof(char) * 11)))
    return (0);
  j = 0;
  while (j < 10)
    lon[j++] = string[i++];
  lon[j] = '\0';
  return (atof(lon));
}

double to_degrees(double radians) {
    return (radians * (180.0 / M_PI));
    }

double  to_radians(double degrees) 
{
  return (degrees * M_PI / 180.0);
}

double  get_angle(float lat1, float lon1, float lat2, float lon2)
{
  double  angle;
  double  dLon;
  double  y;
  double  x;

  dLon = (lon2 - lon1);
  y = sin(dLon) * cos(lat2);
  x = (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2)) * cos(dLon);
  angle = atan2(y, x);
  angle = to_degrees(angle);
  angle = fmod((angle + 360), 360);
  angle = fabs(180 - angle);
  return (angle);
}

