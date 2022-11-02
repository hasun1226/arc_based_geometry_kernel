#pragma once

#define EPS 0.001
#define M_PI 3.14159265358979323846

typedef struct Point {
	double x;
	double y;
} Point;

typedef struct Vector {
	double x;
	double y;
	double z;
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
	ArcStruct(int t, Point c, double r, Angle s, Angle e, bool b = true) {
		type = t;
		center = c;
		radius_sq = r;
		radius = sqrt(r);
		startAngle = s;
		endAngle = e;
		concave = b;
	}
	ArcStruct(int t, Point c, double r, Point sp, Point ep, Angle s, Angle e, bool b = true) {
		type = t;
		center = c;
		radius_sq = r;
		radius = sqrt(r);
		startP = sp;
		endP = ep;
		startAngle = s;
		endAngle = e;
		concave = b;
	}
	int type;	// 0 = point, 1 = line, 2 = arc
	Point center;
	double radius;
	double radius_sq;
	Point startP;
	Point endP;
	Angle startAngle;
	Angle endAngle;
	bool concave = true;
} ArcStruct;

struct ArcPoint : ArcStruct {
	ArcPoint() {
	}
	ArcPoint(Point c) : ArcStruct(c, 0) {}
};

bool isEqual(double a, double b);
bool isEqual(Point a, Point b);

double Angle2radian(Angle a);
Angle radian2Angle(double angle);

Point transform(const Point rotAxis, Angle a, Point translate);
Point calcPoint(ArcStruct arc, Angle angle);