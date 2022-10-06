#include <math.h>
#include <algorithm>
#include "geometry.h"

using namespace std;

Point getMidpoint(Point A, Point B) {
    return Point{ (A.x + B.x) / 2, (A.y + B.y) / 2 };
}

double getDistance(Point A, Point B) {
    double tx = A.x - B.x;
    double ty = A.y - B.y;
    return sqrt(tx * tx + ty * ty);
}
double getDistanceSquared(Point A, Point B) {
    double tx = A.x - B.x;
    double ty = A.y - B.y;
    return (tx * tx + ty * ty);
}

bool isEqual(double a, double b) {
    return abs(a - b) < EPS;
}

bool isEqual(Angle a, Angle b) {
    return (a.quad == b.quad) && (isEqual(a.angle, b.angle));
}

bool isEqual(Point a, Point b) {
    return isEqual(a.x, b.x) && isEqual(a.y, b.y);
}

Point normalize(Point target) {
    double denom = sqrt(target.x * target.x + target.y * target.y);
    Point ret{ 0, 0 };
    if (denom != 0.0f) {
        ret.x = target.x / denom;
        ret.y = target.y / denom;
    }
    return ret;
}

Vector normalize(Vector target) {
    double denom = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    Vector ret{ 0, 0, 0 };
    if (denom != 0.0f) {
        ret.x = target.x / denom;
        ret.y = target.y / denom;
        ret.z = target.z / denom;
    }
    return ret;
}

Point add(Point a, Point b) {
    return Point{ a.x + b.x, a.y + b.y };
}

Vector add(Vector a, Vector b) {
    return Vector{ a.x + b.x, a.y + b.y, a.z + b.z };
}

Point subtract(Point a, Point b) {
    return Point{ a.x - b.x, a.y - b.y };
}

Vector subtract(Vector a, Vector b) {
    return Vector{ a.x - b.x, a.y - b.y, a.z - b.z };
}

Point scale(Point a, double b) {
    return Point{ a.x * b, a.y * b };
}

Vector scale(Vector a, double b) {
    return Vector{ a.x * b, a.y * b, a.z * b };
}

Vector Point2Vec(Point a) {
    return Vector{ a.x, a.y, 0 };
}

Point Vec2Point(Vector a) {
    if (a.z == 0) {
        return Point{ a.x, a.y };
    }
    return Point{ a.x / a.z, a.y / a.z };
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

double calc_angle(Point first, Point second, Point center) {
    double x1 = first.x - center.x;
    double y1 = first.y - center.y;
    double x2 = second.x - center.x;
    double y2 = second.y - center.y;

    double angle = acos((x1 * x2 + y1 * y2) / (x1 * x1 + y1 * y1));
    return angle;
}

Angle calc_angleStr(Point first, Point center) {
    double angle = atan2(first.y - center.y, first.x - center.x);	// radians
    return radian2Angle(angle);
}

bool angle_sort(pair<Angle, Point> const& a, pair<Angle, Point> const& b) {
    Angle first = a.first;
    Angle second = b.first;
    return (second.isGreater(first));
}

bool angle_sort_desc(pair<Angle, Point> const& a, pair<Angle, Point> const& b) {
    Angle first = a.first;
    Angle second = b.first;
    return (first.isGreater(second));
}