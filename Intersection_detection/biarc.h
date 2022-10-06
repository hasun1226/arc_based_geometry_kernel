#pragma once
#include "geometry.h"

vector<Point> doArcIntersect(ArcStruct first, ArcStruct second);

Point calc_junction_point(Point p1, Point p2, Vector t1, Vector t2);

pair<ArcStruct, ArcStruct> computeBiarc(Point p1, Point p2, Vector t1, Vector t2);