#include <math.h>
#include "biarc.h"

using namespace std;

bool isOnArc(ArcStruct arc, double angle) {
    double start = arc.startAngle;
    double end = arc.endAngle;
    bool ret = 0;
    if (angle < 0) angle += 2 * M_PI;
    if (arc.concave) {
        if (start > end) {
            ret = (angle >= end) && (start >= angle);
        } else {
            ret = (angle >= end) || (start >= angle);
        }
    } else {
        if (start > end) {
            ret = (angle >= start) || (end >= angle);
        } else {
            ret = (angle >= start) && (end >= angle);
        }
    }
    return ret;
}

vector<Point> doArcIntersect(ArcStruct first, ArcStruct second) {
    vector<Point> ret;
    if (first.center.x > second.center.x) {
        // swap
        ArcStruct first_copy = first;
        first = second;
        second = first_copy;
    }
    double r1_sq = first.radius_sq;
    double r2_sq = second.radius_sq;

    // check if the end points lie on the arc
    Point startP1 = first.startP;
    Point endP1 = first.endP;
    Point startP2 = second.startP;
    Point endP2 = second.endP;
    /*if (isEqual(first.center, second.center) && isEqual(r1_sq, r2_sq)
        && isEqual(startAngle1, startAngle2) && isEqual(endAngle1, endAngle2)) {
        // TO DO: Handle the same arcs
    }*/
    bool b1 = isEqual(getDistanceSquared(startP1, second.center), r2_sq);  // s1 is on second arc
    bool b2 = isEqual(getDistanceSquared(startP2, first.center), r1_sq);  // s2 is on first arc
    bool b3 = isEqual(getDistanceSquared(endP1, second.center), r2_sq);  // e1 is on second arc
    bool b4 = isEqual(getDistanceSquared(endP2, first.center), r1_sq);  // e2 is on first arc
    // make sure no duplicate is added
    if (b1) {
        ret.push_back(startP1);
    }
    if (b2 && (!b1 || !isEqual(startP1, startP2))) {
        ret.push_back(startP2);
    }
    if (b3 && (!b1 || !isEqual(startP1, endP1)) && (!b2 || !isEqual(endP1, startP2))) {
        ret.push_back(endP1);
    }
    if (b4 && (!b1 || !isEqual(endP2, startP1)) && (!b2 || !isEqual(endP2, startP2)) && (!b3 || !isEqual(endP2, endP1))) {
        ret.push_back(endP2);
    }
    if (ret.size() == 2) return ret;    // intersections are at most 2

    Point c2 = subtract(second.center, first.center);
    double d_sq = c2.x * c2.x + c2.y * c2.y;
    double mid_sq = 2 * sqrt(r1_sq * r2_sq);
    double r1Ar2_sq = r1_sq + r2_sq + mid_sq;
    double r1Sr2_sq = r1_sq + r2_sq - mid_sq;

    // check if there is no intersection between the circles
    if ((d_sq > r1Ar2_sq) || (d_sq < r1Sr2_sq)) {
        return ret;
    }
    double A = d_sq + r1_sq - r2_sq;
    double denom = 2 * d_sq;
    double B = 2 * denom * r1_sq;
    double in_sqrt = B - A * A;

    double x_f = A * c2.x;
    double y_f = A * c2.y;

    // check if the intersection is on the arc
    if (in_sqrt < EPS) {
        Point temp_int = Point{ x_f / denom, y_f / denom };
        Point intersection = add(temp_int, first.center);
        if (ret.size() != 0 && isEqual(ret[0], intersection)) {
            return ret;
        }
        double ang1 = atan2(temp_int.y, temp_int.x);
        double ang2 = atan2(temp_int.y - c2.y, temp_int.x - c2.x);
        if (isOnArc(first, ang1) && isOnArc(second, ang2)) {
            ret.push_back(intersection);
        }
    } else {
        double x_s = c2.y * sqrt(in_sqrt);
        double y_s = c2.x * sqrt(in_sqrt);
        Point intersection1 = Point{ (x_f - x_s) / denom, (y_f + y_s) / denom };
        double ang1 = atan2(intersection1.y, intersection1.x);
        double ang2 = atan2(intersection1.y - c2.y, intersection1.x - c2.x);
        intersection1 = add(intersection1, first.center);
        if (isOnArc(first, ang1) && isOnArc(second, ang2)) {
            if (ret.size() == 0) {
                ret.push_back(intersection1);
            } else if (!isEqual(ret[0], intersection1)) {
                ret.push_back(intersection1);
            }
        }
        if (ret.size() == 2) return ret;    // intersections are at most 2
        Point intersection2 = Point{ (x_f + x_s) / denom, (y_f - y_s) / denom };
        ang1 = atan2(intersection2.y, intersection2.x);
        ang2 = atan2(intersection2.y - c2.y, intersection2.x - c2.x);
        intersection2 = add(intersection2, first.center);
        if (isOnArc(first, ang1) && isOnArc(second, ang2)) {
            if (ret.size() == 0) {
                ret.push_back(intersection2);
            } else if (!isEqual(ret[0], intersection2)) {
                ret.push_back(intersection2);
            }
        }
    }
    return ret;
}

Point calc_junction_point(Point p1, Point p2, Vector t1, Vector t2) {
    // Bisector of p1, p2 : a1X + b1Y + c1Z = 0
    double a1 = p1.x - p2.x;
    double b1 = p1.y - p2.y;
    double c1 = -0.5 * (a1 * (p1.x + p2.x) + b1 * (p1.y + p2.y));

    // Bisector of t1, t2 : a2X + b2Y + c2Z = 0
    double t1x = p1.x + t1.x;
    double t1y = p1.y + t1.y;
    double t2x = p2.x + t2.x;
    double t2y = p2.y + t2.y;
    double a2 = t1x - t2x;
    double b2 = t1y - t2y;
    double c2 = -0.5 * (a2 * (t1x + t2x) + b2 * (t1y + t2y));

    // The intersection of two bisector lines : Cramer's rule
    double denom = a1 * b2 - a2 * b1;
    Point j_center = Point{ (b1 * c2 - b2 * c1) / denom, (c1 * a2 - c2 * a1) / denom };

    Point midpoint = getMidpoint(p1, p2);
    Point direction = subtract(midpoint, j_center);
    Point n_dir = normalize(direction);
    double radius = getDistance(p1, j_center);
    return add(j_center, scale(n_dir, radius));
}

Point calc_arc_center(Point p, Point j, Vector t) {
    // Perpendicular line of p : a1X + b1Y + c1Z = 0
    double a1 = t.x;
    double b1 = t.y;
    double c1 = -1 * (p.x * t.x + p.y * t.y);

    // Bisector of p, j : a1X + b1Y + c1Z = 0
    double a2 = p.x - j.x;
    double b2 = p.y - j.y;
    double c2 = -0.5 * (a2 * (p.x + j.x) + b2 * (p.y + j.y));

    // The intersection of two lines : Cramer's rule
    double denom = a1 * b2 - a2 * b1;
    return Point{ (b1 * c2 - b2 * c1) / denom, (c1 * a2 - c2 * a1) / denom };
}

bool isClockwise(Point point, Vector tangent, Point center) {
    Point p1 = subtract(point, center);
    Point p2 = Point{ p1.x + tangent.x, p1.y + tangent.y };
    double det = p1.x * p2.y - p1.y * p2.x;
    if (det < 0) {
        return true;
    } else {
        // in case of det = 0, the arcs have different direction
        return false;
    }
}

pair<ArcStruct, ArcStruct> computeBiarc(Point p1, Point p2, Vector t1, Vector t2) {
    // calculate the junction point of the two arcs
    Point j_point = calc_junction_point(p1, p2, t1, t2);

    // calculate arc center for each arc
    Point arc_cent1 = calc_arc_center(p1, j_point, t1);
    Point arc_cent2 = calc_arc_center(p2, j_point, t2);

    // calculate angle for each arc
    double angle1 = atan2(p1.y - arc_cent1.y, p1.x - arc_cent1.x);
    double angle2 = atan2(j_point.y - arc_cent1.y, j_point.x - arc_cent1.x);
    double angle3 = atan2(j_point.y - arc_cent2.y, j_point.x - arc_cent2.x);
    double angle4 = atan2(p2.y - arc_cent2.y, p2.x - arc_cent2.x);

    double r1 = getDistance(p1, arc_cent1);
    double r2 = getDistance(p2, arc_cent2);
    ArcStruct first{ arc_cent1, r1, getDistanceSquared(p1, arc_cent1), p1, j_point, angle1, angle2, isClockwise(p1, t1, arc_cent1) };
    ArcStruct second{ arc_cent2, r2, getDistanceSquared(p2, arc_cent2), j_point, p2, angle3, angle4, isClockwise(p2, t2, arc_cent2) };
    return make_pair(first, second);
}
