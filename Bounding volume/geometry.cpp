#include <math.h>
#include <algorithm>
#include "geometry.h"

using namespace std;

bool isEqual(double a, double b) {
    return abs(a - b) < EPS;
}

bool isEqual(Point a, Point b) {
    return isEqual(a.x, b.x) && isEqual(a.y, b.y);
}

double Angle2radian(Angle a) {
    return a.quad * M_PI / 2 + a.angle;
}

Angle radian2Angle(double angle) {
    if (angle < 0)  angle += 2 * M_PI;
    int quad = 0;
    if (angle < 2 * M_PI) {
        quad = trunc(angle * 2 / M_PI);
    }
    return Angle(quad, fmod(angle, M_PI / 2));
}

Point transform(const Point rotAxis, Angle a, Point translate) {
    double angle = Angle2radian(a);
    return Point{ rotAxis.x * cos(angle) - rotAxis.y * sin(angle) + translate.x, rotAxis.y * cos(angle) + rotAxis.x * sin(angle) + translate.y };
}

Point calcPoint(ArcStruct arc, Angle angle) {
    return transform(Point{ arc.radius, 0 }, angle, arc.center);
}

double helper_min(double a, double b){
    // choose the minimum value only if it is not -1
    // if one of a and b is -1, choose the other
    if (isEqual(a, -1)) {
        return b;
    }
    else if (isEqual(b, -1)) {
        return a;
    }
    else {
        return min(a, b);
    }
}