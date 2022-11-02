#include <GL/glut.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "geometry.h"
#include "boundingVolume.h"

using namespace std;

GLsizei width = 640, height = 480;
vector<Volume> bvh;
vector<ArcStruct> arcs;
double scale_factor = 1.0f;
double x_translate = 0, y_translate = 0;
double step = 100;
bool first = false, second = false, third = false;

void init() {
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0, width, 0, height);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_LINE_SMOOTH, GL_NICEST);
    glHint(GL_POINT_SMOOTH, GL_NICEST);
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
    case '1':
        first = !first;
        break;
    case '2':
        second = !second;
        break;
    case '3':
        third = !third;
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

void drawArc(ArcStruct arc) {
    double startAngle = Angle2radian(arc.startAngle);
    double endAngle = Angle2radian(arc.endAngle);

    glPushMatrix();
    glTranslatef(arc.center.x, arc.center.y, 0);  // center
    glScalef(arc.radius, arc.radius, 1);    // radius

    glBegin(GL_LINE_STRIP);
    if (arc.concave) {
        if (startAngle < endAngle) {
            startAngle += 2 * M_PI;
        }
        for (double theta = startAngle; theta > endAngle; theta -= M_PI / 30) {
            glVertex2f(cos(theta), sin(theta));
        }
    }
    else {
        // draw counter-clockwise
        if (startAngle > endAngle) {
            endAngle += 2 * M_PI;
        }
        for (double theta = startAngle; theta < endAngle; theta += M_PI / 30) {
            glVertex2f(cos(theta), sin(theta));
        }
    }
    glVertex2f(cos(endAngle), sin(endAngle));
    glEnd();
    glPopMatrix();
}

void drawLine(Point start, Point end) {
    glBegin(GL_LINES);
    glVertex2f(start.x, start.y);
    glVertex2f(end.x, end.y);
    glEnd();
}

void drawBoundingVolume(vector<Volume>& bvh, int start, int end, bool draw, bool finalLevel) {
    for (int i = start; i < end; i++) {
        if (draw) {
            if (bvh[i].support0 >= 0) {
                if (bvh[i].support1 > 0) {
                    double y_val = min(bvh[i].support1 * multiplier - bvh[i].maximum.x, bvh[i].maximum.y);
                        drawLine(Point{ bvh[i].support0, bvh[i].minimum.y }, Point{ bvh[i].support0, y_val });
                }
                else {
                    double y_val = max(bvh[i].support7 * multiplier + bvh[i].maximum.x, bvh[i].minimum.y);
                        drawLine(Point{ bvh[i].support0, bvh[i].maximum.y }, Point{ bvh[i].support0, y_val });
                }
            }
            if (bvh[i].support1 >= 0)
                drawLine(Point{ bvh[i].support1 * multiplier - bvh[i].maximum.y, bvh[i].maximum.y },
                    Point{ bvh[i].maximum.x, bvh[i].support1 * multiplier - bvh[i].maximum.x });
            if (bvh[i].support2 >= 0) {
                if (bvh[i].support1 > 0) {
                    double x_val = min(bvh[i].support1 * multiplier - bvh[i].maximum.y, bvh[i].maximum.x);
                        drawLine(Point{ bvh[i].minimum.x, bvh[i].support2 }, Point{ x_val, bvh[i].support2 });
                }
                else {
                    double x_val = max(bvh[i].maximum.y - bvh[i].support3 * multiplier, bvh[i].minimum.x);
                        drawLine(Point{ bvh[i].maximum.x, bvh[i].support2 }, Point{ x_val, bvh[i].support2 });
                }
            }
            if (bvh[i].support3 >= 0)
                drawLine(Point{ bvh[i].maximum.y - bvh[i].support3 * multiplier, bvh[i].maximum.y },
                    Point{ bvh[i].minimum.x, bvh[i].support3 * multiplier + bvh[i].minimum.x });
            if (bvh[i].support4 >= 0) {
                if (bvh[i].support5 > 0) {
                    double y_val = max(bvh[i].support5 * multiplier - bvh[i].minimum.x, bvh[i].minimum.y);
                        drawLine(Point{ bvh[i].support4, y_val }, Point{ bvh[i].support4, bvh[i].maximum.y });
                }
                else {
                    double y_val = min(bvh[i].support3 * multiplier + bvh[i].minimum.x, bvh[i].maximum.y);
                    drawLine(Point{ bvh[i].support4, bvh[i].minimum.y }, Point{ bvh[i].support4, y_val });
                }
            }
            if (bvh[i].support5 >= 0)
                drawLine(Point{ bvh[i].minimum.x, bvh[i].support5 * multiplier - bvh[i].minimum.x },
                    Point{ bvh[i].support5 * multiplier - bvh[i].minimum.y, bvh[i].minimum.y });
            if (bvh[i].support6 >= 0) {
                if (bvh[i].support5 > 0) {
                    double x_val = max(bvh[i].support5 * multiplier - bvh[i].minimum.y, bvh[i].minimum.x);
                        drawLine(Point{ x_val, bvh[i].support6 }, Point{ bvh[i].maximum.x, bvh[i].support6 });
                }
                else {
                    double x_val = min(bvh[i].minimum.y - bvh[i].support7 * multiplier, bvh[i].maximum.x);
                    drawLine(Point{ x_val, bvh[i].support6 }, Point{ bvh[i].minimum.x, bvh[i].support6 });
                }
            }
            if (bvh[i].support7 >= 0) {
                drawLine(Point{ bvh[i].minimum.y - bvh[i].support7 * multiplier, bvh[i].minimum.y },
                    Point{ bvh[i].maximum.x, bvh[i].maximum.x + bvh[i].support7 * multiplier });
            }
        }
        if (!finalLevel) {
            if (i % 2 == 1) {
                bvh.push_back(Volume{ bvh[i - 1], bvh[i] });
            }
        }
    }
}

void display_callback() {
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(8.0);
    glLineWidth(4.0);

    glPushMatrix();
    glTranslatef(x_translate, y_translate, 0);
    glScalef(scale_factor, scale_factor, 1);

    glColor3ub(0, 0, 0);
    for (int i = 0; i < arcs.size(); i++) {
        drawArc(arcs[i]);
        Volume bv = Volume{ arcs[i] };
        bvh.push_back(bv);  // initial bounding volumes at the lowest level
    }

    glLineWidth(2.0);
    glColor3ub(80, 80, 80);
    int size_init = bvh.size();
    drawBoundingVolume(bvh, 0, size_init, first, false);

    int size_sec = bvh.size();
    glColor3ub(120, 120, 120);
    drawBoundingVolume(bvh, size_init, size_sec, second, false);

    int size_th = bvh.size();
    glColor3ub(200, 200, 200);
    drawBoundingVolume(bvh, size_sec, size_th, third, true);

    glPopMatrix();
    bvh.clear();
    glutSwapBuffers();
}

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(width, height);
    glutCreateWindow("Bounding volume");

    init();

    // the arcs are constructed so that each arc is fully included in a quadrant
    arcs.push_back(ArcStruct{ 2, Point{891.04, -100}, 772.606 * 772.606, radian2Angle(2.67794), radian2Angle(2.59013) });
    arcs.push_back(ArcStruct{ 2, Point{480.511, 152.509}, 290.637 * 290.637, radian2Angle(2.59009), radian2Angle(2.35619) });
    arcs.push_back(ArcStruct{ 2, Point{395.829, 237.191}, 170.878 * 170.878, radian2Angle(2.35619), radian2Angle(2.10537) });
    arcs.push_back(ArcStruct{ 2, Point{350, 314.6}, 80.9198 * 80.9198, radian2Angle(2.10541), radian2Angle(1.5708) });
    arcs.push_back(ArcStruct{ 2, Point{350, 314.6}, 80.9207 * 80.9207, radian2Angle(1.5708), radian2Angle(1.03619) });
    arcs.push_back(ArcStruct{ 2, Point{304.168, 237.188}, 170.882 * 170.882, radian2Angle(1.03621), radian2Angle(0.785398) });
    arcs.push_back(ArcStruct{ 2, Point{219.504, 152.524}, 290.615 * 290.615, radian2Angle(0.785398), radian2Angle(0.551486) });
    arcs.push_back(ArcStruct{ 2, Point{-190.904, -100}, 772.455 * 772.455, radian2Angle(0.551478), radian2Angle(0.463648) });

    arcs.push_back(ArcStruct{ 2, Point{891.04, 550}, 772.606 * 772.606, radian2Angle(3.69305), radian2Angle(3.60524) });
    arcs.push_back(ArcStruct{ 2, Point{480.511, 298.029}, 290.637 * 290.637, radian2Angle(3.92699), radian2Angle(3.69309) });
    arcs.push_back(ArcStruct{ 2, Point{395.829, 213.347}, 170.878 * 170.878, radian2Angle(4.17781), radian2Angle(3.92699) });
    arcs.push_back(ArcStruct{ 2, Point{350, 135.938}, 80.9198 * 80.9198, radian2Angle(4.71238), radian2Angle(4.17777) });
    arcs.push_back(ArcStruct{ 2, Point{350, 135.938}, 80.9207 * 80.9207, radian2Angle(5.24699), radian2Angle(4.71239) });
    arcs.push_back(ArcStruct{ 2, Point{304.168, 213.347}, 170.882 * 170.882, radian2Angle(5.49779), radian2Angle(5.24697) });
    arcs.push_back(ArcStruct{ 2, Point{219.504, 298.029}, 290.615 * 290.615, radian2Angle(5.73170), radian2Angle(5.49779) });
    arcs.push_back(ArcStruct{ 2, Point{-190.904, 550}, 772.455 * 772.455, radian2Angle(5.81954), radian2Angle(5.73171) });

    glutReshapeFunc(reshape_callback);
    glutKeyboardFunc(keyboard_callback);
    glutSpecialFunc(keyPressed);
    glutDisplayFunc(display_callback);
    glutMainLoop();

    return 0;
}