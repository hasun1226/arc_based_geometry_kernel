#include <math.h>
#include <algorithm>
#include "bezier.h"
#include "biarc.h"

Point evaluate(const bezierCubic* curve, const double t) {
	const double t_inv = 1 - t;
	const double t_inv_sq = t_inv * t_inv;
	const double t_sq = t * t;
	const double b0 = t_inv_sq * t_inv;
	const double b1 = 3 * t_inv_sq * t;
	const double b2 = 3 * t_inv * t_sq;
	const double b3 = t_sq * t;
	Point ret = Point{ 0, 0 };
	ret.x += (curve->control_pts[0].x * b0 + curve->control_pts[1].x * b1 + curve->control_pts[2].x * b2 + curve->control_pts[3].x * b3);
	ret.y += (curve->control_pts[0].y * b0 + curve->control_pts[1].y * b1 + curve->control_pts[2].y * b2 + curve->control_pts[3].y * b3);
	return ret;
}

Vector getTangent(const bezierCubic* curve, const double t) {
	const double t_inv = 1 - t;
	const double t_inv_sq = t_inv * t_inv;
	const double t_sq = t * t;
	Vector ret = Vector{ 0, 0 , 0 };
	ret.x += curve->control_pts[1].x * t_inv * (t_inv - 2 * t);
	ret.y += curve->control_pts[1].y * t_inv * (t_inv - 2 * t);

	ret.x += curve->control_pts[2].x * t * (2 * t_inv - t);
	ret.y += curve->control_pts[2].y * t * (2 * t_inv - t);

	ret.x += curve->control_pts[3].x * t_sq - curve->control_pts[0].x * t_inv_sq;
	ret.y += curve->control_pts[3].y * t_sq - curve->control_pts[0].y * t_inv_sq;
	return normalize(ret);
}

std::vector<double> findExtremePt(const bezierCubic* curve, char ch) {
	double a, b, c;
	std::vector<double> ret;

	if (ch == 'x') {
		a = 3 * (curve->control_pts[1].x - curve->control_pts[0].x);
		b = 3 * (curve->control_pts[2].x - curve->control_pts[1].x);
		c = 3 * (curve->control_pts[3].x - curve->control_pts[2].x);
	}
	else {
		a = 3 * (curve->control_pts[1].y - curve->control_pts[0].y);
		b = 3 * (curve->control_pts[2].y - curve->control_pts[1].y);
		c = 3 * (curve->control_pts[3].y - curve->control_pts[2].y);
	}

	double denom = a - 2 * b + c;
	double aSb = a - b;
	double det = aSb * aSb - (aSb - b + c) * a;
	if (isEqual(denom, 0)) {
		// linear formula
		if (!isEqual(aSb, 0)) ret.push_back(round((a / (2 * aSb)) * 100) / 100);
	}
	else {
		if (det >= 0) {
			// return only if return value is real number
			double ret1 = (aSb - sqrt(det)) / denom;
			if (ret1 >= 0 && ret1 <= 1) {
				ret.push_back(round(ret1 * 100) / 100);
			}
			if (det > EPS) {
				// not a double root
				double ret2 = (aSb + sqrt(det)) / denom;
				if (ret2 >= 0 && ret2 <= 1) {
					ret.push_back(round(ret2 * 100) / 100);
				}
			}
		}
	}
	return ret;
}

std::vector<double> findInflectionPt(const bezierCubic* curve) {
	std::vector<double> ret;
	double ax = 3 * (curve->control_pts[1].x - curve->control_pts[0].x);
	double bx = 3 * (curve->control_pts[2].x - curve->control_pts[1].x);
	double cx = 3 * (curve->control_pts[3].x - curve->control_pts[2].x);
	double ay = 3 * (curve->control_pts[1].y - curve->control_pts[0].y);
	double by = 3 * (curve->control_pts[2].y - curve->control_pts[1].y);
	double cy = 3 * (curve->control_pts[3].y - curve->control_pts[2].y);

	double denom = 2 * ((by - ay) * (ax - cx) - (bx - ax) * (ay - cy));
	double b = (2 * by - cy) * ax - (2 * bx - cx) * ay;
	double c = by * ax - bx * ay;
	double det = b * b - denom * c;

	if (isEqual(denom, 0)) {
		// linear formula
		if (!isEqual(b, 0)) ret.push_back(round((c / b) * 100) / 100);
	}
	else {
		if (det >= 0) {
			// return only if return value is real number
			double ret1 = (b - sqrt(det)) / denom;
			if (ret1 >= 0 && ret1 <= 1) {
				ret.push_back(round(ret1 * 100) / 100);
			}
			if (det > EPS) {
				// not a double root
				double ret2 = (b + sqrt(det)) / denom;
				if (ret2 >= 0 && ret2 <= 1) {
					ret.push_back(round(ret2 * 100) / 100);
				}
			}
		}
	}
	return ret;
}

double getApproxError(const bezierCubic* curve, double start, double end, Point j_p) {
	double interval = end - start;
	double b = floor(interval * 100.0) / 100.0;
	double min_error = FLT_MAX;
	for (double t = start + 0.01; t < end; t += 0.01) {
		double t_inv = 1.0f - t;
		double t_inv_sq = t_inv * t_inv;
		double t_sq = t * t;
		double b0 = t_inv_sq * t_inv;
		double b1 = 3 * t_inv_sq * t;
		double b2 = 3 * t_inv * t_sq;
		double b3 = t_sq * t;
		Point p{ curve->control_pts[0].x * b0 + curve->control_pts[1].x * b1 + curve->control_pts[2].x * b2 + curve->control_pts[3].x * b3,
				curve->control_pts[0].y * b0 + curve->control_pts[1].y * b1 + curve->control_pts[2].y * b2 + curve->control_pts[3].y * b3 };
		double error = getDistance(p, j_p);
		if (min_error > error) {
			min_error = error;
		}
	}
	return abs(min_error);
}

vector<double> curveSegmentation(const bezierCubic* curve) {
	vector<double> x = findExtremePt(curve, 'x');
	vector<double> y = findExtremePt(curve, 'y');
	vector<double> seg = findInflectionPt(curve);
	seg.insert(seg.end(), y.begin(), y.end());
	seg.insert(seg.end(), x.begin(), x.end());
	sort(seg.begin(), seg.end());
	seg.insert(seg.begin(), 0);
	seg.insert(seg.end(), 1);

	// approximate the bezier curve with arc
	double error = 2.0;
	while (error > 1.0) {
		error = 0;
		for (int i = 0; i < seg.size() - 1; i++) {
			Point pt1 = evaluate(curve, seg[i]);
			Vector tan1 = getTangent(curve, seg[i]);
			Point pt2 = evaluate(curve, seg[i + 1]);
			Vector tan2 = getTangent(curve, seg[i + 1]);
			Point j_p = calc_junction_point(pt1, pt2, tan1, tan2);
			error += getApproxError(curve, seg[i], seg[i + 1], j_p);
		}
		error = error / seg.size();
		if (error > 1.0) {
			// subdivide the arc approximation
			int length = seg.size();
			int offset = 0;
			for (int j = 0; j < length - 1; j++) {
				int new_j = j + offset;
				if (seg[new_j + 1] - seg[new_j] > 0.01) {
					float new_val = roundf((seg[new_j + 1] + seg[new_j]) / 2 * 100) / 100;
					seg.insert(seg.begin() + new_j + 1, new_val);
					offset++;
				}
			}
			if (length == seg.size()) {
				// no increase in the segments
				break;
			}
		}
	}
	return seg;
}