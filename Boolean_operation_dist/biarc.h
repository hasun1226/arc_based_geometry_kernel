#pragma once

#include <vector>
#include "geometry.h"

vector<Point> doArcIntersect(ArcStruct first, ArcStruct second);

Point calc_junction_point(Point p1, Point p2, Vector t1, Vector t2);

Point calc_arc_center(Point p, Point j, Vector t);

double calc_angle(Point first, Point second, Point center);

Angle calc_angleStr(Point first, Point center);

bool isClockwise(Point f, Vector s, Point c);

pair<vector<ArcStruct>, vector<ArcStruct>> boolean(ArcStruct first, ArcStruct second, char opcode);

pair<vector<ArcStruct>, vector<ArcStruct>> boolean_arr(vector<ArcStruct> first, vector<ArcStruct> second, char opcode);