#pragma once

#include <vector>
#include "geometry.h"

using namespace std;

typedef struct bezierCubic {
	vector<Point> control_pts;
} bezierCubic;

Point evaluate(const bezierCubic* curve, const double t);

Vector getTangent(const bezierCubic* curve, const double t);

vector<double> findExtremePt(const bezierCubic* curve, char c);

vector<double> findInflectionPt(const bezierCubic* curve);

double getApproxError(const bezierCubic* curve, double start, double end, Point j_p);

vector<double> curveSegmentation(const bezierCubic* curve);