#include <GL/glut.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include <algorithm>
#include "geometry.h"
#include "bezier.h"
#include "biarc.h"

using namespace std;

GLsizei width = 800, height = 640;
vector<ArcStruct> arcs1, arcs2;
vector<LineSeg> lines1, lines2, line_int_segs;
vector<Point> arc_ints, line_ints;
bezierCubic curve1, curve2;
char mode = 'a';
double scale_factor = 1;
double x_translate = 0;
double y_translate = 0; 
double lineseg = 50;
double step = 100;
bool mode_int = true;

void drawArc(ArcStruct arc) {
    double startAngle = arc.startAngle;
    double endAngle = arc.endAngle;

    glPushMatrix();
    glTranslated(arc.center.x, arc.center.y, 0);  // center
    glScaled(arc.radius, arc.radius, 1);    // radius

    glBegin(GL_LINE_STRIP);
    if (arc.concave) {
        if (startAngle < endAngle) {
            startAngle += 2 * M_PI;
        }
        for (double theta = startAngle; theta > endAngle; theta -= M_PI / 10000) {
            glVertex2d(cos(theta), sin(theta));
        }
    }
    else {
        // draw counter-clockwise
        if (startAngle > endAngle) {
            endAngle += 2 * M_PI;
        }
        for (double theta = startAngle; theta < endAngle; theta += M_PI / 10000) {
            glVertex2d(cos(theta), sin(theta));
        }
    }
    glVertex2d(cos(endAngle), sin(endAngle));
    glEnd();
    glPopMatrix();
}

void printResult() {
    // curve segmentation
    vector<double> seg1 = curveSegmentation(&curve1);
    vector<double> seg2 = curveSegmentation(&curve2);

    for (int i = 0; i < seg1.size() - 1; i++) {
        Point pt1 = evaluate(&curve1, seg1[i]);
        Vector tan1 = getTangent(&curve1, seg1[i]);
        Point pt2 = evaluate(&curve1, seg1[i + 1]);
        Vector tan2 = getTangent(&curve1, seg1[i + 1]);
        pair<ArcStruct, ArcStruct> biarc = computeBiarc(pt1, pt2, tan1, tan2);
        arcs1.push_back(biarc.first);
        arcs1.push_back(biarc.second);
    }
    for (int i = 0; i < seg2.size() - 1; i++) {
        Point pt1 = evaluate(&curve2, seg2[i]);
        Vector tan1 = getTangent(&curve2, seg2[i]);
        Point pt2 = evaluate(&curve2, seg2[i + 1]);
        Vector tan2 = getTangent(&curve2, seg2[i + 1]);
        pair<ArcStruct, ArcStruct> biarc = computeBiarc(pt1, pt2, tan1, tan2);
        arcs2.push_back(biarc.first);
        arcs2.push_back(biarc.second);
    }

    cout << "Curve 1: ";
    for (int i = 0; i < curve1.control_pts.size(); i++) {
        cout << "(" << curve1.control_pts[i].x << ", " << curve1.control_pts[i].y << ")";
        if (i != curve1.control_pts.size() - 1) cout << " & ";
        else cout << endl;
    }
    cout << "Curve 2: ";
    for (int i = 0; i < curve2.control_pts.size(); i++) {
        cout << "(" << curve2.control_pts[i].x << ", " << curve2.control_pts[i].y << ")";
        if (i != curve2.control_pts.size() - 1) cout << " & ";
        else cout << endl;
    }

    // time Arc intersection
    cout << "Number of arcs: " << arcs1.size() << " vs " << arcs2.size() << endl;
    auto a_start = chrono::steady_clock::now();
    sort(arcs1.begin(), arcs1.end(), x_sort);
    sort(arcs2.begin(), arcs2.end(), x_sort);
    for (int i = 0; i < arcs1.size(); i++) {
        for (int j = 0; j < arcs2.size(); j++) {
            if (arcs1[i].startP.x <= arcs2[j].startP.x) {
                if (arcs1[i].endP.x < arcs2[j].startP.x) {
                    break;
                }
                vector<Point> temp_int = doArcIntersect(arcs1[i], arcs2[j]);
                arc_ints.insert(arc_ints.end(), temp_int.begin(), temp_int.end());
            }
            else if (arcs1[i].startP.x <= arcs2[j].endP.x) {
                if (arcs1[i].endP.x < arcs2[j].startP.x) {
                    break;
                }
                vector<Point> temp_int = doArcIntersect(arcs1[i], arcs2[j]);
                arc_ints.insert(arc_ints.end(), temp_int.begin(), temp_int.end());
            }
        }
    }
    auto a_end = chrono::steady_clock::now();

    string arc_int_str = "";
    if (arc_ints.size() == 0) {
        cout << "no intersection" << endl;
    } else {
        for (int i = 0; i < arc_ints.size(); i++) {
            arc_int_str += "Point(" + to_string(arc_ints[i].x) + ", " + to_string(arc_ints[i].y) + ")";
            if (i + 1 < arc_ints.size()) arc_int_str += ", ";
        }
    }

    cout << "Arc intersection:\n\t"
        << arc_int_str << "\n\t"
        << chrono::duration_cast<chrono::nanoseconds>(a_end - a_start).count() << " ns" << endl;

    // time Line intersection
    cout << "Number of lines: " << lines1.size() << endl;
    auto l_start = chrono::steady_clock::now();
    sort(lines1.begin(), lines1.end(), x_sort_line);
    sort(lines2.begin(), lines2.end(), x_sort_line);
    for (int i = 0; i < lines1.size(); i++) {
        for (int j = 0; j < lines2.size(); j++) {
            if (lines1[i].start.x <= lines2[j].start.x) {
                if (lines1[i].end.x < lines2[j].start.x) {
                    break;
                }
                vector<LineSeg> temp_int = doLineIntersect(lines1[i], lines2[j]);
                line_int_segs.insert(line_int_segs.end(), temp_int.begin(), temp_int.end());
            }
            else if (lines1[i].start.x <= lines2[j].end.x) {
                if (lines1[i].end.x < lines2[j].start.x) {
                    break;
                }
                vector<LineSeg> temp_int = doLineIntersect(lines1[i], lines2[j]);
                line_int_segs.insert(line_int_segs.end(), temp_int.begin(), temp_int.end());
            }
        }
    }
    auto l_end = chrono::steady_clock::now();

    string line_int_str = "";
    if (line_int_segs.size() == 0) {
        cout << "no intersection" << endl;
    }
    else {
        for (int i = 0; i < line_int_segs.size(); i++) {
            if (isEqual(line_int_segs[i].start, line_int_segs[i].end)) {
                line_ints.push_back(line_int_segs[i].start);
                line_int_str += "Point(" + to_string(line_int_segs[i].start.x) + ", " + to_string(line_int_segs[i].start.y) + ")";
            }
            else {
                line_int_str += "LineSeg(" + to_string(line_int_segs[i].start.x) + ", " + to_string(line_int_segs[i].start.y) + ") ~ ("
                    + to_string(line_int_segs[i].end.x) + ", " + to_string(line_int_segs[i].end.y) + ")";
            }
            if (i + 1 < line_int_segs.size()) line_int_str += ", ";
        }
    }

    cout << "Line intersection:\n\t"
        << line_int_str << "\n\t"
        << chrono::duration_cast<chrono::nanoseconds>(l_end - l_start).count() << " ns" << endl;

    cout << "Time difference between Line and Arc:\n\t"
        << chrono::duration_cast<chrono::nanoseconds>(l_end - l_start).count() - chrono::duration_cast<chrono::nanoseconds>(a_end - a_start).count()
        << " ns\n=======================================================" << endl;
}

void init() {
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, width, 0, height);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH, GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH, GL_NICEST);

    curve1.control_pts.push_back(Point{ 300, 300 });
    curve1.control_pts.push_back(Point{ 400, 600 });
    curve1.control_pts.push_back(Point{ 500, 600 });
    curve1.control_pts.push_back(Point{ 600, 300 });

    curve2.control_pts.push_back(Point{ 300, 300 });
    curve2.control_pts.push_back(Point{ 400.01, 600.01 });
    curve2.control_pts.push_back(Point{ 499.99, 600.01 });
    curve2.control_pts.push_back(Point{ 600, 300 });

    // init line
    Point nullPt = Point{ -1, -1 };
    for (int i = 0; i <= lineseg; i++) {
        const double t = i / lineseg;
        Point pt = evaluate(&curve1, t);
        Point pt2 = evaluate(&curve2, t);
        if (i == 0) {
            lines1.push_back(LineSeg{ pt, nullPt });
            lines2.push_back(LineSeg{ pt2, nullPt });
        } else {
            lines1.back().end = pt;
            lines2.back().end = pt2;
            if (i < lineseg) {
                lines1.push_back(LineSeg{ pt, nullPt });
                lines2.push_back(LineSeg{ pt2, nullPt });
            }
        }
    }
}

void display_callback() {
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(8.0);
    glLineWidth(4.0);

    // draw control points for bezier curve
    glPushMatrix();
    glTranslated(x_translate, y_translate, 0);
    glScaled(scale_factor, scale_factor, 1);
    if (mode == 'a' || mode == 'b') {
        glColor3ub(0, 0, 0);
        glBegin(GL_POINTS);
        for (int i = 0; i < curve1.control_pts.size(); i++) {
            glColor3ub(255, 0, 0);
            glVertex2d(curve1.control_pts[i].x, curve1.control_pts[i].y);
            glColor3ub(0, 0, 255);
            glVertex2d(curve2.control_pts[i].x, curve2.control_pts[i].y);
        }
        glEnd();

        // draw Bezier curve
        glColor3ub(255, 0, 0);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i <= 1000; i++) {
            const double t = i / (double)1000;
            Point pt = evaluate(&curve1, t);
            glVertex2d(pt.x, pt.y);
        }
        glEnd();
        glColor3ub(0, 0, 255);
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i <= 100; i++) {
            const double t = i / (double)100;
            Point pt = evaluate(&curve2, t);
            glVertex2d(pt.x, pt.y);
        }
        glEnd();
    }

    // draw line segments approximating bezier curve
    if (mode == 'a' || mode == 'l') {
        glColor3ub(0, 0, 0);
        glBegin(GL_LINES);
        glColor3ub(255, 0, 0);
        for (int i = 0; i < lines1.size(); i++) {
            glVertex2d(lines1[i].start.x, lines1[i].start.y);
            glVertex2d(lines1[i].end.x, lines1[i].end.y);
        }
        glColor3ub(0, 0, 255);
        for (int i = 0; i < lines2.size(); i++) {
            glVertex2d(lines2[i].start.x, lines2[i].start.y);
            glVertex2d(lines2[i].end.x, lines2[i].end.y);
        }
        glEnd();

        // draw line intersections
        if (mode_int) {
            glColor3ub(0, 0, 0);
            glBegin(GL_LINES);
            for (int i = 0; i < line_int_segs.size(); i++) {
                glVertex2d(line_int_segs[i].start.x, line_int_segs[i].start.y);
                glVertex2d(line_int_segs[i].end.x, line_int_segs[i].end.y);
            }
            glEnd();
            glBegin(GL_POINTS);
            for (int i = 0; i < line_ints.size(); i++) {
                glVertex2d(line_ints[i].x, line_ints[i].y);
            }
            glEnd();
        }
    }

    // draw arcs
    if (mode == 'a' || mode == 'c') {
        for (int i = 0; i < arcs1.size(); i++) {
            glColor3ub(255, 0, 0);
            drawArc(arcs1[i]);
        }
        for (int i = 0; i < arcs2.size(); i++) {
            glColor3ub(0, 0, 255);
            drawArc(arcs2[i]);
        }

        // draw arc intersections
        if (mode_int) {
            glColor3ub(0, 0, 0);
            glBegin(GL_POINTS);
            for (int i = 0; i < arc_ints.size(); i++) {
                glVertex2d(arc_ints[i].x, arc_ints[i].y);
            }
            glEnd();
        }
    }
    glPopMatrix();

    // draw text
    glRasterPos2f(10, 10);
    string str = "";
    str += " (" + to_string(scale_factor) + " / " + to_string(x_translate) + ", " + to_string(y_translate) + ")";
    for (int i = 0; i < str.length(); i++) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }

    glutSwapBuffers();
}

void reshape_callback(GLint nw, GLint nh) {
    width = nw;
    height = nh;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);
}

void keyboard_callback(unsigned char key, int x, int y) {
    switch (key) {
    case 'a': case 'A':
        mode = 'a';
        break;
    case 'b': case 'B':
        mode = 'b'; // bezier
        break;
    case 'c': case 'C':
        mode = 'c'; // arc
        break;
    case 'l': case 'L':
        mode = 'l'; // line
        break;
    case 's': case 'S':
        mode_int = !mode_int;   // intersection point
        break;
    case '+':
        scale_factor = scale_factor * 2;
        break;
    case '-':
        scale_factor = scale_factor * 0.5;
        break;
    case '0':
        scale_factor = 1.0f;
        x_translate = 0;
        y_translate = 0;
        break;
    case (27): exit(0); break;
    default: break;
    }
    glutPostRedisplay();
}

void keyPressed(int key, int x1, int y1) {
    if (key == GLUT_KEY_UP) {
        y_translate += step;
    }
    if (key == GLUT_KEY_DOWN) {
        y_translate -= step;
    }
    if (key == GLUT_KEY_RIGHT) {
        x_translate += step;
    }
    if (key == GLUT_KEY_LEFT) {
        x_translate -= step;
    }
    glutPostRedisplay();
}

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
    glutInitWindowSize(width, height);
    glutCreateWindow("Intersection detection precision");

    init();
    printResult();

    glutReshapeFunc(reshape_callback);
    glutDisplayFunc(display_callback);
    glutKeyboardFunc(keyboard_callback);
    glutSpecialFunc(keyPressed);
    glutMainLoop();

    return 0;
}