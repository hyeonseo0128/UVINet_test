//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions
#include <string.h>

#ifndef LATLONGCONV
#define LATLONGCONV

void LLtoUTM(int ReferenceEllipsoid, float Lat, float Long, float *UTMNorthing, float *UTMEasting, char* UTMZone);
void UTMtoLL(int ReferenceEllipsoid, float UTMNorthing, float UTMEasting, const char* UTMZone, float *Lat,  float *Long );
char UTMLetterDesignator(float Lat);

typedef struct ellipsoid {
    int id;
    char *ellipsoidName;
    float EquatorialRadius;
    float eccentricitySquared;
} Ellipsoid;

#endif
