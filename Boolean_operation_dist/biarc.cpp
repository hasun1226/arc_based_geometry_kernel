#include <math.h>
#include <algorithm>
#include "biarc.h"

using namespace std;

bool isOnArc(ArcStruct arc, double angle) {
	double start = Angle2radian(arc.startAngle);
	double end = Angle2radian(arc.endAngle);
	bool isOnArc = false;
	if (angle < 0.0f) angle += 2 * M_PI;
	if (arc.concave) {
		if (start > end) {
			isOnArc = (angle >= end) && (start >= angle);
		}
		else {
			isOnArc = (angle >= end) || (start >= angle);
		}
	}
	else {
		if (start > end) {
			isOnArc = (angle >= start) || (end >= angle);
		}
		else {
			isOnArc = (angle >= start) && (end >= angle);
		}
	}
	return isOnArc;
}

vector<Point> doArcIntersect(ArcStruct first, ArcStruct second) {
	vector<Point> ret;
	if (first.center.x > second.center.x) {
		// swap
		ArcStruct first_copy = first;
		first = second;
		second = first_copy;
	}
	double r1 = first.radius;
	double r2 = second.radius;
	double r1_sq = first.radius_sq;
	double r2_sq = second.radius_sq;

	// check if the end points are the same
	Point startP1 = calcPoint(first, first.startAngle);
	Point endP1 = calcPoint(first, first.endAngle);
	Point startP2 = calcPoint(second, second.startAngle);
	Point endP2 = calcPoint(second, second.endAngle);

	if (isEqual(first.center, second.center) && isEqual(r1_sq, r2_sq)
		&& isEqual(first.startAngle, second.startAngle) && isEqual(first.endAngle, second.endAngle)) {
		// To do : handle the same arcs
	}
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

	double r1Ar2 = r1 + r2;
	double r1Sr2 = r1 - r2;
	Point c2 = Point{ second.center.x - first.center.x, second.center.y - first.center.y };
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
	}
	else {
		double x_s = c2.y * sqrt(in_sqrt);
		double y_s = c2.x * sqrt(in_sqrt);
		Point intersection1 = Point{ (x_f - x_s) / denom, (y_f + y_s) / denom };
		double ang1 = atan2(intersection1.y, intersection1.x);
		double ang2 = atan2(intersection1.y - c2.y, intersection1.x - c2.x);
		intersection1 = add(intersection1, first.center);
		if (isOnArc(first, ang1) && isOnArc(second, ang2)) {
			if (ret.size() == 0) {
				ret.push_back(intersection1);
			}
			else if (!isEqual(ret[0], intersection1)) {
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
			}
			else if (!isEqual(ret[0], intersection2)) {
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
	double c1 = -0.5f * (a1 * (p1.x + p2.x) + b1 * (p1.y + p2.y));

	// Bisector of t1, t2 : a2X + b2Y + c2Z = 0
	double t1x = p1.x + t1.x;
	double t1y = p1.y + t1.y;
	double t2x = p2.x + t2.x;
	double t2y = p2.y + t2.y;
	double a2 = t1x - t2x;
	double b2 = t1y - t2y;
	double c2 = -0.5f * (a2 * (t1x + t2x) + b2 * (t1y + t2y));

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
	double c1 = -1.0f * (p.x * t.x + p.y * t.y);

	// Bisector of p, j : a1X + b1Y + c1Z = 0
	double a2 = p.x - j.x;
	double b2 = p.y - j.y;
	double c2 = -0.5f * (a2 * (p.x + j.x) + b2 * (p.y + j.y));

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
	}
	else {
		// in case of det = 0, the arcs have different direction
		return false;
	}
}

vector<pair<Angle, Point>> order(vector<pair<Angle, Point>> v, Angle start, Angle end, bool concave) {
	if (concave) {
		if (start.isGreater(end)) {
			vector<pair<Angle, Point>> ret = v;
			sort(ret.begin(), ret.end(), angle_sort_desc);
			return ret;
		} else {
			vector<pair<Angle, Point>> temp_s, temp_e;
			for (int i = 0; i < v.size(); i++) {
				if (start.isGreater(v[i].first)) {
					temp_s.push_back(v[i]);
				} else {
					temp_e.push_back(v[i]);
				}
			}
			sort(temp_s.begin(), temp_s.end(), angle_sort_desc);
			sort(temp_e.begin(), temp_e.end(), angle_sort_desc);
			temp_s.insert(temp_s.end(), temp_e.begin(), temp_e.end());
			return temp_s;
		}
	} else {
		if (start.isGreater(end)) {
			vector<pair<Angle, Point>> temp_s, temp_e;
			for (int i = 0; i < v.size(); i++) {
				if (start.isGreater(v[i].first)) {
					temp_e.push_back(v[i]);
				} else {
					temp_s.push_back(v[i]);
				}
			}
			sort(temp_s.begin(), temp_s.end(), angle_sort);
			sort(temp_e.begin(), temp_e.end(), angle_sort);
			temp_s.insert(temp_s.end(), temp_e.begin(), temp_e.end());
			return temp_s;
		} else {
			vector<pair<Angle, Point>> ret = v;
			sort(ret.begin(), ret.end(), angle_sort);
			return ret;
		}
	}
}

pair<bool, bool> checkRemoveArc(char opcode, Point s1, Point s2, Point e1, Point e2, ArcStruct first, ArcStruct second) {
	bool rem1 = false, rem2 = false;
	if (isEqual(s1, s2) && isEqual(e1, e2)) {
		// case 0 : check curvature
		if (first.concave) {
			if (first.radius < second.radius) (opcode == 'u') ? rem1 = true : rem2 = true;
		}
		else {
			if (first.radius > second.radius) (opcode == 'u') ? rem1 = true : rem2 = true;
		}
		if (!rem1 && !rem2 && opcode != 'd') {
			// Nothing removed because radius is the same, remove concave one for union
			if (first.concave != second.concave) {
				if (first.concave) (opcode == 'u') ? rem1 = true : rem2 = true;
				else(opcode == 'u') ? rem2 = true : rem1 = true;
			}
		}
	}
	else if (isEqual(e1, e2)) {
		// case 1 : (e1 = e2) remove the point on the left
		if (((e1.x - s1.x) * (s2.y - s1.y) - (e1.y - s1.y) * (s2.x - s1.x)) > 0) {
			// s2 is on the left of the first segment
			if (opcode != 'd') (opcode == 'u') ? rem2 = true : rem1 = true;
		}
		else {
			(opcode == 'u') ? rem1 = true : rem2 = true;
			if (opcode == 'd') rem1 = true;
		}
	}
	else if (isEqual(s1, s2)) {
		// case 2 : (s1 = s2) remove the point on the left
		if (((e1.x - s1.x) * (e2.y - s1.y) - (e1.y - s1.y) * (e2.x - s1.x)) > 0) {
			// e2 is on the left of the first segment
			if (opcode != 'd') (opcode == 'u') ? rem2 = true : rem1 = true;
		}
		else {
			(opcode == 'u') ? rem1 = true : rem2 = true;
			if (opcode == 'd') rem1 = true;
		}
	}
	return make_pair(rem1, rem2);
}

pair<vector<ArcStruct>, vector<ArcStruct>> boolean(ArcStruct first, ArcStruct second, char opcode) {
	pair<vector<ArcStruct>, vector<ArcStruct>> ret;
	vector<Point> intersections = doArcIntersect(first, second);	// Find intersection
	vector<ArcStruct> first_arcs;
	vector<ArcStruct> second_arcs;
	if (intersections.size() == 0) {
		if (opcode == 'u') {
			first_arcs.push_back(first);
			second_arcs.push_back(second);
		} else if (opcode == 'd') first_arcs.push_back(first);
	} else {
		// Check if the intersection is the end point
		Point startP1 = calcPoint(first, first.startAngle);
		Point endP1 = calcPoint(first, first.endAngle);
		Point startP2 = calcPoint(second, second.startAngle);
		Point endP2 = calcPoint(second, second.endAngle);

		vector<pair<Angle, Point>> split_1, split_2;	// intersection order
		for (int i = 0; i < intersections.size(); i++) {
			Point intersection = intersections[i];
			if (!isEqual(intersection, startP1) && !isEqual(intersection, endP1)) {
				// intersection is not the endpoint of the first arc: split first arc on the intersection
				Angle angle = calc_angleStr(intersection, first.center);
				split_1.push_back(make_pair(angle, intersection));
			}
			if (!isEqual(intersection, startP2) && !isEqual(intersection, endP2)) {
				// intersection is not the endpoint of the second arc: split second arc on the intersection
				Angle angle = calc_angleStr(intersection, second.center);
				split_2.push_back(make_pair(angle, intersection));
			}
		}

		vector<pair<Angle, Point>> split1 = order(split_1, first.startAngle, first.endAngle, first.concave);
		bool same1 = isEqual(split1[0].second, split_1[0].second);
		split1.insert(split1.begin(), make_pair(first.startAngle, startP1));
		split1.push_back(make_pair(first.endAngle, endP1));
		if (same1) {
			// First arc is the same
			split_1.insert(split_1.begin(), make_pair(first.startAngle, startP1));
			split_1.push_back(make_pair(first.endAngle, endP1));
		} else {
			split_1.insert(split_1.begin(), make_pair(first.endAngle, endP1));
			split_1.push_back(make_pair(first.startAngle, startP1));
		}
		vector<pair<Angle, Point>> split2 = order(split_2, second.startAngle, second.endAngle, second.concave);
		bool same2 = isEqual(split2[0].second, split_2[0].second);
		split2.insert(split2.begin(), make_pair(second.startAngle, startP2));
		split2.push_back(make_pair(second.endAngle, endP2));
		if (same2) {
			// second arc is the same
			split_2.push_back(make_pair(second.endAngle, endP2));
			split_2.insert(split_2.begin(), make_pair(second.startAngle, startP2));
		} else {
			split_2.insert(split_2.begin(), make_pair(second.endAngle, endP2));
			split_2.push_back(make_pair(second.startAngle, startP2));
		}
		
		// order from startP to endP
		for (int i = 0; i < split1.size() - 1; i++) {
			ArcStruct new_arc{ 2, first.center, first.radius_sq, split1[i].first, split1[i + 1].first, first.concave };
			first_arcs.push_back(new_arc);
		}
		for (int i = 0; i < split2.size() - 1; i++) {
			ArcStruct new_arc{ 2, second.center, second.radius_sq, split2[i].first, split2[i + 1].first, second.concave };
			second_arcs.push_back(new_arc);
		}

		// Remove the included arcs
		vector<int> remove1, remove2;
		double mid_sq = 2 * sqrt(first.radius_sq * second.radius_sq);
		double r1Sr2_sq = first.radius_sq + second.radius_sq - mid_sq;
		double d_sq = getDistanceSquared(first.center, second.center);
		if (d_sq <= r1Sr2_sq) {
			// Check inclusion of arcs
			if (first.concave == second.concave) {
				if (first.concave) {
					if (first.radius_sq < second.radius_sq && opcode != 'd') {
						(opcode == 'u') ? second_arcs.clear() : first_arcs.clear();
					} else {
						(opcode == 'u') ? first_arcs.clear() : second_arcs.clear();
					}
				} else {
					if (first.radius_sq < second.radius_sq) {
						(opcode == 'i' || opcode == 'd') ? second_arcs.clear() : first_arcs.clear();
					} else if (opcode != 'd') {
						(opcode == 'i') ? first_arcs.clear() : second_arcs.clear();
					}
				}
			}
		} else {
			for (int i = 0; i < intersections.size(); i++) {
				Point intersection = intersections[i];
				// First arc
				Point former1 = split_1[i].second;
				Point mid1 = split_1[i + 1].second;
				Point latter1 = split_1[i + 2].second;
				Vector diff1 = (same1) ? Point2Vec(subtract(mid1, former1)) : Point2Vec(subtract(former1, mid1));
				Vector diff1_1 = (same1) ? Point2Vec(subtract(latter1, mid1)) : Point2Vec(subtract(mid1, latter1));
				// Second arc
				Point former2 = split_2[i].second;
				Point mid2 = split_2[i + 1].second;
				Point latter2 = split_2[i + 2].second;
				Vector diff2 = (same2) ? Point2Vec(subtract(mid2, former2)) : Point2Vec(subtract(former2, mid2));
				Vector diff2_1 = (same2) ? Point2Vec(subtract(latter2, mid2)) : Point2Vec(subtract(mid2, latter2));
			
				// i vs j
				Point s1 = former1, e1 = mid1, s2 = former2, e2 = mid2;
				if (!same1) s1 = mid1, e1 = former1;
				if (!same2) s2 = mid2, e2 = former2;
				pair<bool, bool> res = checkRemoveArc(opcode, s1, s2, e1, e2, first, second);
				if (res.first) remove1.push_back(i);
				if (res.second) remove2.push_back(i);

				// i vs j+1
				if (same2) s2 = mid2, e2 = latter2;
				else s2 = latter2, e2 = mid2;
				res = checkRemoveArc(opcode, s1, s2, e1, e2, first, second);
				if (res.first) remove1.push_back(i);
				if (res.second) remove2.push_back(i+1);

				// i+1 vs j
				if (same1) s1 = mid1, e1 = latter1;
				else s1 = latter1, e1 = mid1;
				if (same2) s2 = former2, e2 = mid2;
				else s2 = mid2, e2 = former2;
				res = checkRemoveArc(opcode, s1, s2, e1, e2, first, second);
				if (res.first) remove1.push_back(i+1);
				if (res.second) remove2.push_back(i);

				// i+1 vs j+1
				if (same2) s2 = mid2, e2 = latter2;
				else s2 = latter2, e2 = mid2;
				res = checkRemoveArc(opcode, s1, s2, e1, e2, first, second);
				if (res.first) remove1.push_back(i+1);
				if (res.second) remove2.push_back(i+1);
			}
			remove1.erase(unique(remove1.begin(), remove1.end()), remove1.end());
			sort(remove1.begin(), remove1.end(), greater<int>());
			for (int i = 0; i < remove1.size(); i++) {
				int remove = (same1) ? remove1[i] : first_arcs.size() - remove1[i] - 1;
				first_arcs.erase(first_arcs.begin() + remove);
			}
			remove2.erase(unique(remove2.begin(), remove2.end()), remove2.end());
			sort(remove2.begin(), remove2.end(), greater<int>());
			for (int i = 0; i < remove2.size(); i++) {
				int remove = (same2) ? remove2[i] : second_arcs.size() - remove2[i] - 1;
				second_arcs.erase(second_arcs.begin() + remove);
			}
		}
	}
	ret = make_pair(first_arcs, second_arcs);
	return ret;
}

pair<vector<ArcStruct>, vector<ArcStruct>> boolean_arr(vector<ArcStruct> first, vector<ArcStruct> second, char opcode) {
	vector<ArcStruct> first_ret, second_ret;
	vector<pair<pair<int, int>, Point>> intersections;
	for (int i = 0; i < first.size(); i++) {
		for (int j = 0; j < second.size(); j++) {
			vector<Point> temp_int = doArcIntersect(first[i], second[j]);
			// Check if the intersection is the end point
			for (int k = 0; k < temp_int.size(); k++) {
				Point intersection = temp_int[k];
				Point startP1 = calcPoint(first[i], first[i].startAngle);
				Point endP1 = calcPoint(first[i], first[i].endAngle);
				Point startP2 = calcPoint(second[j], second[j].startAngle);
				Point endP2 = calcPoint(second[j], second[j].endAngle);
				if (!isEqual(intersection, startP1) && !isEqual(intersection, endP1)) {
					intersections.push_back(make_pair(make_pair(i, j), intersection));
				}
			}
		}
	}
	vector<ArcStruct> first_arcs, second_arcs;
	vector<pair<int, pair<Angle, Angle>>> end1, end2;
	if (intersections.size() == 0) {
		if (opcode == 'u') {
			first_ret.insert(first_ret.begin(), first.begin(), first.end());
			second_ret.insert(second_ret.begin(), second.begin(), second.end());
		} else if (opcode == 'd') first_ret.insert(first_ret.begin(), first.begin(), first.end());
	} else {
		for (int i = 0; i < intersections.size(); i++) {
			Point intersection = intersections[i].second;
			pair<int, int> ind = intersections[i].first;
			pair<vector<ArcStruct>, vector<ArcStruct>> temp = boolean(first[ind.first], second[ind.second], opcode);
			end1.push_back(make_pair(ind.first, make_pair(temp.first[0].startAngle, temp.first.back().endAngle)));
			end2.push_back(make_pair(ind.second, make_pair(temp.second[0].startAngle, temp.second.back().endAngle)));
			first_arcs.insert(first_arcs.begin(), temp.first.begin(), temp.first.end());
			second_arcs.insert(second_arcs.begin(), temp.second.begin(), temp.second.end());
		}
		first_ret.insert(first_ret.begin(), first_arcs.begin(), first_arcs.end());
		second_ret.insert(second_ret.begin(), second_arcs.begin(), second_arcs.end());
		// Check the missing arcs
		for (int i = 1; i < end1.size(); i++) {
			int startInd = end1[i].first;
			int endInd = end1[i-1].first;
			double startAngle = Angle2radian(end1[i].second.first);
			double endAngle = Angle2radian(end1[i-1].second.second);
			if (isEqual(startAngle, Angle2radian(first[startInd].startAngle)) || isEqual(endAngle, Angle2radian(first[endInd].endAngle))) {
				for (int j = end1[i - 1].first + 1; j < end1[i].first; j++) {
					first_ret.push_back(first[j]);
				}
			}
		}
		for (int i = 1; i < end2.size(); i++) {
			int startInd = end2[i].first;
			int endInd = end2[i - 1].first;
			double startAngle = Angle2radian(end2[i].second.first);
			double endAngle = Angle2radian(end2[i - 1].second.second);
			if (isEqual(startAngle, Angle2radian(second[startInd].startAngle)) || isEqual(endAngle, Angle2radian(second[endInd].endAngle))) {
				for (int j = end2[i - 1].first + 1; j < end2[i].first; j++) {
					second_ret.push_back(second[j]);
				}
			}
		}
	}
	return make_pair(first_ret, second_ret);
}
