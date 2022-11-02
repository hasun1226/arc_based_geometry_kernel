#include <cmath>
#include "geometry.h"

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