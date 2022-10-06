#pragma once

#include <vector>

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
	double z;
} Vector;

typedef struct LineSeg {
	Point start;
	Point end;
} LineSeg;


typedef struct ArcStruct {
	Point center;
	double radius;
	double radius_sq;
	Point startP;
	Point endP;
	double startAngle; // radians
	double endAngle; // radians
	bool concave = true;
} ArcStruct;

Point getMidpoint(Point A, Point B);

double getDistance(Point A, Point B);
double getDistanceSquared(Point A, Point B);

bool isEqual(double a, double b);
bool isEqual(Point a, Point b);

Point normalize(Point target);
Vector normalize(Vector target);

Point add(Point a, Point b);
Vector add(Vector a, Vector b);

Point subtract(Point a, Point b);
Vector subtract(Vector a, Vector b);

Point scale(Point a, double b);

Vector Point2Vec(Point a);
Point Vec2Point(Vector a);

vector<LineSeg> doLineIntersect(LineSeg l1, LineSeg l2);

bool x_sort(ArcStruct const& a, ArcStruct const& b);
bool x_sort_line(LineSeg const& a, LineSeg const& b);