#include "../../include/plane_detector.h"

float Pdetect::get_planeangle(float a, float b, float c, float d)
{
    float dot;
    float base_mag;
    float mag;
    float PI;
    PI=3.1415;

    dot=               baseplane.a * a
                     + baseplane.b * b
                     + baseplane.c * c;
    base_mag= sqrt(baseplane.a *baseplane.a
                     + baseplane.b *baseplane.b
                     + baseplane.c *baseplane.c);
    mag=      sqrt(a*a
                     + b*b
                     + c*c);
    float planeangle;
    planeangle = abs(180-acos(dot/(base_mag*mag))*180.0/PI);
    return planeangle;
}

float Pdetect::get_relative_angle(Pdetect::Plane pri, Pdetect::Plane newplane)
{
    float dot;
    float base_mag;
    float mag;
    float PI;
    PI=3.1415;

    dot=               pri.a * newplane.a
                     + pri.b * newplane.b
                     + pri.c * newplane.c;
    base_mag= sqrt(pri.a *pri.a
                     + pri.b *pri.b
                     + pri.c *pri.c);
    mag=      sqrt(newplane.a*newplane.a
                     + newplane.b*newplane.b
                     + newplane.c*newplane.c);
    float planeangle;
    planeangle = abs(acos(dot/(base_mag*mag))*180.0/PI);
    return planeangle;
}
