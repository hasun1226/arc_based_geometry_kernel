#pragma once

#define M_PI 3.14159265358979323846
#define EPS 0.00001

using namespace std;

typedef struct Point {
	double x;
	double y;
} Point;

typedef struct Vector {
	double x;
	double y;
	double z = 0;
} Vector;

typedef struct Angle {
	Angle() {}
	Angle(int q, double a) {
		quad = q;
		angle = a;
	}
	bool isGreater(Angle b) {
		if (quad == b.quad) {
			return (angle > b.angle);
		}
		else {
			return (quad > b.quad);
		}
	}
	int quad;
	double angle;
} Angle;

struct zeroA : Angle {
	zeroA() : Angle(0, 0) {}
};

struct fullA : Angle {
	fullA() : Angle(3, 90 - EPS) {}
};

typedef struct ArcStruct {
	ArcStruct() {
	}
	ArcStruct(Point c, int type = 0) {
		type = 0;
		center = c;
		radius = 0.0f;
		startAngle = zeroA{};
		endAngle = fullA{};
		concave = true;
	}
	ArcStruct(int t, Point c, double r, Angle s, Angle e) {
		type = t;
		center = c;
		radius_sq = r;
		radius = sqrt(r);
		startAngle = s;
		endAngle = e;
		concave = true;
	}
	ArcStruct(int t, Point c, double r, Angle s, Angle e, bool b) {
		type = t;
		center = c;
		radius_sq = r;
		radius = sqrt(r);
		startAngle = s;
		endAngle = e;
		concave = b;
	}
	int type;	// 0 = point, 1 = line, 2 = arc
	Point center;
	double radius;
	double radius_sq;
	Angle startAngle;
	Angle endAngle;
	bool concave = true;
} ArcStruct;

struct ArcPoint : ArcStruct {
	ArcPoint() {
	}
	ArcPoint(Point c) : ArcStruct(c, 0) {}
};

Point getMidpoint(Point A, Point B);

double getDistance(Point A, Point B);
double getDistanceSquared(Point A, Point B);

bool isEqual(double a, double b);
bool isEqual(Angle a, Angle b);
bool isEqual(Point a, Point b);

Point normalize(Point target);
Vector normalize(Vector target);

Point add(Point a, Point b);
Vector add(Vector a, Vector b);

Point subtract(Point a, Point b);
Vector subtract(Vector a, Vector b);

Point scale(Point a, double b);
Vector scale(Vector a, double b);

Vector Point2Vec(Point a);
Point Vec2Point(Vector a);

double Angle2radian(Angle a);
Angle radian2Angle(double angle);

Point transform(const Point rotAxis, Angle a, Point translate);

Point calcPoint(ArcStruct arc, Angle angle);
double calc_angle(Point first, Point second, Point center);
Angle calc_angleStr(Point first, Point center);

bool angle_sort(pair<Angle, Point> const& a, pair<Angle, Point> const& b);
bool angle_sort_desc(pair<Angle, Point> const& a, pair<Angle, Point> const& b);