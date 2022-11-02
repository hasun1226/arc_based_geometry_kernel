#pragma once

#include <math.h>
#include <algorithm>
#include "geometry.h"

using namespace std;
double multiplier = sqrt(2);

typedef struct Volume {
	Volume() {
	}
	Volume(ArcStruct arc) {
		Angle sAng = arc.startAngle;
		Angle eAng = arc.endAngle;
		if (arc.startAngle.isGreater(arc.endAngle)) {
			sAng = arc.endAngle;
			eAng = arc.startAngle;
		}
		Point sP = calcPoint(arc, sAng);
		Point eP = calcPoint(arc, eAng);
		double max_x = max(sP.x, eP.x);
		double max_y = max(sP.y, eP.y);
		double min_x = min(sP.x, eP.x);
		double min_y = min(sP.y, eP.y);
		double sAng_r = Angle2radian(sAng);
		double eAng_r = Angle2radian(eAng);
		if (arc.endAngle.quad == 0) {
			// the arc resides in the 1st quadrant (Epsilons are used to handle numerical error)
			if (isEqual(eAng_r, M_PI / 2)) support0 = max_x;
			if (sAng_r + EPS < M_PI / 4) {
				support0 = max_x;
				support1 = abs(eP.x * cos(M_PI / 4) + eP.y * sin(M_PI / 4));
			}
			else {
				support1 = abs(sP.x * cos(M_PI / 4) + sP.y * sin(M_PI / 4));
			}			
			if (eAng_r - EPS > M_PI / 4) {
				support2 = max_y;
				support6 = min_y;
			}
			else support4 = min_x;
			if (M_PI / 2 - eAng_r < sAng_r) {
				support5 = abs(eP.x * cos(5 * M_PI / 4) + eP.y * sin(5 * M_PI / 4));
				support6 = min_y;
			}
			else {
				support4 = min_x;
				support5 = abs(sP.x * cos(5 * M_PI / 4) + sP.y * sin(5 * M_PI / 4));
			}
		}
		else if (arc.endAngle.quad == 1) {
			// the arc resides in the 2nd quadrant
			if (isEqual(eAng_r, M_PI)) support6 = min_y;
			if (sAng_r + EPS < 3 * M_PI / 4) {
				support6 = min_y;
				support3 = abs(eP.x * cos(3 * M_PI / 4) + eP.y * sin(3 * M_PI / 4));
			}
			else {
				support3 = abs(sP.x * cos(3 * M_PI / 4) + sP.y * sin(3 * M_PI / 4));
			}
			if (eAng_r - EPS > 3 * M_PI / 4) {
				support0 = max_x;
				support4 = min_x;
			}
			else {
				support2 = max_y;
			}
			if (3 * M_PI / 2 - eAng_r < sAng_r) {
				support7 = abs(eP.x * cos(7 * M_PI / 4) + eP.y * sin(7 * M_PI / 4));
				support4 = min_x; 
			}
			else {
				support7 = abs(sP.x * cos(7 * M_PI / 4) + sP.y * sin(7 * M_PI / 4));
				support2 = max_y;
			}
		}
		else if (arc.endAngle.quad == 2) {
			// the arc resides in the 3rd quadrant
			if (isEqual(eAng_r, 3 * M_PI / 2)) support4 = min_x;
			if (sAng_r + EPS < 5 * M_PI / 4) {
				support4 = min_x;
				support5 = abs(eP.x * cos(5 * M_PI / 4) + eP.y * sin(5 * M_PI / 4));
			}
			else {
				support5 = abs(sP.x * cos(5 * M_PI / 4) + sP.y * sin(5 * M_PI / 4));				
			}
			if (eAng_r - EPS > 5 * M_PI / 4) {
				support2 = max_y;
				support6 = min_y;
			}
			else support0 = max_x;
			if (5 * M_PI / 2 - eAng_r < sAng_r) {
				support1 = abs(eP.x * cos(M_PI / 4) + eP.y * sin(M_PI / 4));
				support2 = max_y; 
			}
			else {
				support0 = max_x;
				support1 = abs(sP.x * cos(M_PI / 4) + sP.y * sin(M_PI / 4));
			}
		}
		else if (arc.endAngle.quad == 3) {
			// the arc resides in the 4th quadrant
			/*if (isEqual(eAng_r, 2 * M_PI)) support2 = max_y;
			if (sAng_r + EPS < 7 * M_PI / 4) {
				support2 = max_y; 
				support7 = abs(eP.x * cos(7 * M_PI / 4) + eP.y * sin(7 * M_PI / 4));
			}
			else {
				support7 = abs(sP.x * cos(7 * M_PI / 4) + sP.y * sin(7 * M_PI / 4));
			}
			if (eAng_r - EPS > 7 * M_PI / 4) {
				support0 = max_x;
				support4 = min_x;
			}
			else {
				support6 = min_y;
			}
			if (7 * M_PI / 2 - eAng_r < sAng_r) {
				support3 = abs(eP.x * cos(3 * M_PI / 4) + eP.y * sin(3 * M_PI / 4)); 
				support0 = max_x;
			}
			else {
				support3 = abs(sP.x * cos(3 * M_PI / 4) + sP.y * sin(3 * M_PI / 4));
				support6 = min_y;
			}
			cout << "Quad4: " << sAng_r << " ~ " << eAng_r << " ==> " << support0 << ", " << support2 << ", " << support3 << ", " << support4 << ", " << support6 << ", " << support7 << endl;
			*/
		}
		minimum = Point{ min_x, min_y };
		maximum = Point{ max_x, max_y };
	}
	Volume (Volume v1, Volume v2) {
		// Bounding volume encasing v1 and v2
		support0 = max(v1.support0, v2.support0);
		support1 = max(v1.support1, v2.support1);
		support2 = max(v1.support2, v2.support2);
		support3 = max(v1.support3, v2.support3);
		support4 = min(v1.support4, v2.support4);
		support5 = min(v1.support5, v2.support5);
		support6 = min(v1.support6, v2.support6);
		support7 = min(v1.support7, v2.support7);
		minimum = Point{ min(v1.minimum.x, v2.minimum.x), min(v1.minimum.y, v2.minimum.y) };
		maximum = Point{ max(v1.maximum.x, v2.maximum.x), max(v1.maximum.y, v2.maximum.y) };
		if (support0 > minimum.x && support0 < maximum.x) {
			support0 = -1;
		}
	}
	double support0 = -1;
	double support1 = -1;
	double support2 = -1;
	double support3 = -1;
	double support4 = -1;
	double support5 = -1;
	double support6 = -1;
	double support7 = -1;
	Point minimum, maximum;
} Volume;