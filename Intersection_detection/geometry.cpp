#include <math.h>
#include <vector>
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
    return abs(a - b) <= EPS;
}

bool isEqual(Point a, Point b) {
    return isEqual(a.x, b.x) && isEqual(a.y, b.y);
}

Point normalize(Point target) {
    double denom = target.x * target.x + target.y * target.y;
    Point ret{ 0, 0 };
    if (!isEqual(denom, 0)) {
        denom = sqrt(denom);
        ret.x = target.x / denom;
        ret.y = target.y / denom;
    }
    return ret;
}

Vector normalize(Vector target) {
    double denom = target.x * target.x + target.y * target.y + target.z * target.z;
    Vector ret{ 0, 0, 0 };
    if (!isEqual(denom, 0)) {
        denom = sqrt(denom);
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

Vector Point2Vec(Point a) {
    return Vector{ a.x, a.y, 0 };
}

Point Vec2Point(Vector a) {
    if (isEqual(a.z, 0)) {
        return Point{ a.x, a.y };
    }
    return Point{ a.x / a.z, a.y / a.z };
}

// Given three collinear points p, q, r, the function checks if point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r) {
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}

// To find orientation of ordered triplet (p, q, r)
int orientation(Point p, Point q, Point r) {
    double denom = sqrt(((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y)) * ((q.x - r.x) * (q.x - r.x) + (q.y - r.y) * (q.y - r.y)));
    if (isEqual(denom, 0)) denom = 1;
    double val = ((q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)) / denom;
    if (isEqual(val, 0)) return 0;  // collinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// Returns true if line segment 'p1q1' and 'p2q2' intersect
vector<LineSeg> doLineIntersect(LineSeg seg1, LineSeg seg2) {
    Point p1 = seg1.start, q1 = seg1.end, p2 = seg2.start, q2 = seg2.end;
    vector<LineSeg> ret;
    double o1 = orientation(p1, q1, p2);
    double o2 = orientation(p1, q1, q2);
    double o3 = orientation(p2, q2, p1);
    double o4 = orientation(p2, q2, q1);

    // General case
    if (!isEqual(o1, o2) && !isEqual(o3, o4)) {
        // Line AB represented as a1x + b1y = c1
        double a1 = q1.y - p1.y;
        double b1 = p1.x - q1.x;
        double c1 = a1 * (p1.x) + b1 * (p1.y);

        // Line CD represented as a2x + b2y = c2
        double a2 = q2.y - p2.y;
        double b2 = p2.x - q2.x;
        double c2 = a2 * (p2.x) + b2 * (p2.y);

        double determinant = a1 * b2 - a2 * b1;
        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;
        if ((p1.x - x) * (q1.x - x) <= 0.0f) {
            Point p{ x, y };
            ret.push_back(LineSeg{ p, p });
        }
    }

    // Special Cases
    bool b1 = 0;
    bool b2 = 0;
    bool b3 = 0;
    bool b4 = 0;

    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (isEqual(o1, 0) && onSegment(p1, p2, q1)) {
        b1 = true;
    }

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (isEqual(o2, 0) && onSegment(p1, q2, q1)) {
        b2 = true;
    }

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (isEqual(o3, 0) && onSegment(p2, p1, q2)) {
        b3 = true;
    }

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (isEqual(o4, 0) && onSegment(p2, q1, q2)) {
        b4 = true;
    }

    bool start = b1 || b3;
    bool end = b2 || b4;

    if (start != end) {
        if (start) {
            // start exists, but not end
            Point p = b1 ? p2 : p1;
            ret.push_back(LineSeg{ p, p });
        }
        else {
            // end exists, but not start
            Point p = b2 ? q2 : q1;
            ret.push_back(LineSeg{ p, p });
        }
    }
    else if (start && end) {
        if (b1 && b4) {
            ret.push_back(LineSeg{ p2, q1 });
            Point new1 = { (p2.y - p1.y) * (p1.x - q1.x) / (p1.y - q1.y) + p1.x, p2.y };
            Point new2 = { (q1.y - p2.y) * (p2.x - q2.x) / (p2.y - q2.y) + p2.x, q1.y };
            ret.push_back(LineSeg{ new1, new2 });
        }
        else if (b2 && b3) {
            ret.push_back(LineSeg{ p1, q2 });
            Point new1 = { (p1.y - p2.y) * (p2.x - q2.x) / (p2.y - q2.y) + p2.x, p1.y };
            Point new2 = { (q2.y - p1.y) * (p1.x - q1.x) / (p1.y - q1.y) + p1.x, q2.y };
            ret.push_back(LineSeg{ new1, new2 });
        }
    }
    return ret;
}

bool x_sort(ArcStruct const& a, ArcStruct const& b) {
    return (a.startP.x < b.startP.x);
}

bool x_sort_line(LineSeg const& a, LineSeg const& b) {
    return (a.start.x < b.start.x);
}