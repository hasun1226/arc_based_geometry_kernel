#include <GL/glut.h>
#include <math.h>
#include <vector>
#include "geometry.h"
#include "biarc.h"

using namespace std;

GLsizei width = 800, height = 640;
vector<ArcStruct> arc1, arc2;
pair<vector<ArcStruct>, vector<ArcStruct>> result;
double scale_factor = 1.0f;
double x_translate = 0;
double y_translate = 0;
char opcode = 'x';

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

void display_callback() {
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(8.0);
    glLineWidth(4.0);
    glColor3ub(0, 0, 0);

    // draw arcs
    glPushMatrix();
    glTranslatef(x_translate, y_translate, 0);
    glScalef(scale_factor, scale_factor, 1);
    if (opcode == 'x') {
        glColor3ub(0, 0, 255);
        for (int i = 0; i < arc1.size(); i++) {
            drawArc(arc1[i]);
        }
        glColor3ub(255, 0, 0);
        for (int i = 0; i < arc2.size(); i++) {
            drawArc(arc2[i]);
        }
    }

    glColor3ub(0, 0, 255);
    for (int i = 0; i < result.first.size(); i++) {
        drawArc(result.first[i]);
    }
    glColor3ub(255, 0, 0);
    for (int i = 0; i < result.second.size(); i++) {
        drawArc(result.second[i]);
    }
    
    glColor3ub(0, 0, 0);
    glBegin(GL_POINTS);
    for (int i = 0; i < arc1.size(); i++) {
        for (int j = 0; j < arc2.size(); j++) {
            vector<Point> intersection = doArcIntersect(arc1[i], arc2[j]);	// Find intersection
            for (int k = 0; k < intersection.size(); k++) {
                glVertex2f(intersection[k].x, intersection[k].y);
            }
        }
    }
    glEnd();
    glPopMatrix();
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
    case 'c': case 'C':
        // clear
        result.first.clear();
        result.second.clear();
        opcode = 'x';
        break;
    case 'd': case 'D':
        result = boolean_arr(arc1, arc2, 'd');
        opcode = 'd';
        break;
    case 'i': case 'I':
        opcode = 'i';
        result = boolean_arr(arc1, arc2, 'i');
        break;
    case 'u': case 'U':
        opcode = 'u';
        result = boolean_arr(arc1, arc2, 'u');
        break;
    case '+':
        scale_factor = scale_factor * 1.10;
        break;
    case '-':
        scale_factor = scale_factor * 0.90;
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
        y_translate += 20;
    }
    if (key == GLUT_KEY_DOWN) {
        y_translate -= 20;
    }
    if (key == GLUT_KEY_RIGHT) {
        x_translate += 20;
    }
    if (key == GLUT_KEY_LEFT) {
        x_translate -= 20;
    }
    glutPostRedisplay();
}

int main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_ALPHA);
    glutInitWindowSize(width, height);
    glutCreateWindow("Boolean operator");

    init();    
    Angle angle = radian2Angle(-0.93);

    arc1.push_back(ArcStruct{ 2, Point{ 100, 200 }, 225, Angle{0, 0.93}, angle, false });
    arc1.push_back(ArcStruct{ 2, Point{ 115, 180 }, 100, Angle{1, angle.angle}, angle });
    arc1.push_back(ArcStruct{ 2, Point{ 130, 160 }, 225, Angle{1, angle.angle}, Angle{0, 0.93}, false });
    arc1.push_back(ArcStruct{ 2, Point{ 145, 180 }, 100, Angle{2, 0.93}, Angle{0, 0.93} });
    arc1.push_back(ArcStruct{ 2, Point{ 160, 200 }, 225, Angle{2, 0.93}, Angle{1, angle.angle}, false });
    arc1.push_back(ArcStruct{ 2, Point{ 145, 220 }, 100, angle, Angle{1, angle.angle} });
    arc1.push_back(ArcStruct{ 2, Point{ 130, 240 }, 225, angle, Angle{2, 0.93}, false });
    arc1.push_back(ArcStruct{ 2, Point{ 115, 220 }, 100, Angle{0, 0.93}, Angle{2, 0.93} });

    arc2.push_back(ArcStruct{ 2, Point{ 110, 210 }, 225, Angle{0, 0.93}, angle, false });
    arc2.push_back(ArcStruct{ 2, Point{ 125, 190 }, 100, Angle{1, angle.angle}, angle });
    arc2.push_back(ArcStruct{ 2, Point{ 140, 170 }, 225, Angle{1, angle.angle}, Angle{0, 0.93}, false });
    arc2.push_back(ArcStruct{ 2, Point{ 155, 190 }, 100, Angle{2, 0.93}, Angle{0, 0.93} });
    arc2.push_back(ArcStruct{ 2, Point{ 170, 210 }, 225, Angle{2, 0.93}, Angle{1, angle.angle}, false });
    arc2.push_back(ArcStruct{ 2, Point{ 155, 230 }, 100, angle, Angle{1, angle.angle} });
    arc2.push_back(ArcStruct{ 2, Point{ 140, 250 }, 225, angle, Angle{2, 0.93}, false });
    arc2.push_back(ArcStruct{ 2, Point{ 125, 230 }, 100, Angle{0, 0.93}, Angle{2, 0.93} });
    
    glutReshapeFunc(reshape_callback);
    glutDisplayFunc(display_callback);
    glutKeyboardFunc(keyboard_callback);
    glutSpecialFunc(keyPressed);
    glutMainLoop();
    return 0;
}