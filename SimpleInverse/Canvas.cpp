#include "Canvas.h"
#include <QPainter>
#include <iostream>
#include <QFileInfoList>
#include <QDir>
#include <QMessageBox>
#include <QTextStream>
#include <QDomDocument>
#include <QResizeEvent>
#include <QtWidgets/QApplication>
#include <QDate>
#include <glm/gtc/matrix_transform.hpp>

#define M_PI	3.141592653

glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2) {
	glm::dvec2 dir = center2 - center1;
	double d = glm::length(dir);
	if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
		throw "No intersection";
	}

	double a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0;
	double h = sqrtf(radius1 * radius1 - a * a);

	glm::dvec2 perp(dir.y, -dir.x);
	perp /= glm::length(perp);

	return center1 + dir * a / d + perp * h;
}

glm::dvec2 lineLineIntersection(const glm::dvec2& p1, const glm::dvec2& p2, const glm::dvec2& p3, const glm::dvec2& p4) {
	glm::dvec2 u = p2 - p1;
	glm::dvec2 v = p4 - p3;

	if (glm::length(u) == 0 || glm::length(v) == 0) {
		throw "No intersection";
	}

	double numer = v.x * (p3.y - p1.y) + v.y * (p1.x - p3.x);
	double denom = u.y * v.x - u.x * v.y;

	if (denom == 0.0)  {
		throw "Non intersection";
	}

	double t0 = numer / denom;

	glm::dvec2 ipt = p1 + t0 * u;
	glm::dvec2 tmp = ipt - p3;
	double t1;
	if (glm::dot(tmp, v) > 0.0) {
		t1 = glm::length(tmp) / glm::length(v);
	}
	else {
		t1 = -1.0 * glm::length(tmp) / glm::length(v);
	}

	return p1 + (p2 - p1) * t0;
}

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;
	theta = 0;

	/*
	theta = 140.0 / 180.0 * M_PI;

	std::vector<std::vector<glm::dvec2>> input_points(2);
	input_points[0].push_back(glm::dvec2(500, 200));
	input_points[0].push_back(glm::dvec2(346.791, 328.558));
	input_points[0].push_back(glm::dvec2(504.36, 356.341));
	input_points[0].push_back(glm::dvec2(533.904, 351.132));
	input_points[1].push_back(glm::dvec2(500, 200));
	input_points[1].push_back(glm::dvec2(465.27, 396.962));
	input_points[1].push_back(glm::dvec2(568.116, 519.529));
	input_points[1].push_back(glm::dvec2(578.377, 491.338));

	std::cout << 504.36 + cos(-10.0 / 180.0 * M_PI) * 30 << "," << 356.341 + sin(-10 / 180.0 * M_PI) * 30 << std::endl;
	std::cout << 568.116 + cos(-70.0 / 180.0 * M_PI) * 30 << "," << 519.529 + sin(-70 / 180.0 * M_PI) * 30 << std::endl;

	points.resize(1);
	points[0] = input_points[0][0];

	solveInverse(input_points);
	forwardKinematics(theta);
	*/
}

Canvas::~Canvas() {
}

void Canvas::solveInverse(std::vector<std::vector<glm::dvec2>>& input_points) {
	lengths.clear();
	add_lengths.clear();
	for (int pi = 0; pi < input_points[0].size() - 1; ++pi) {
		lengths.push_back(glm::length(input_points[0][pi + 1] - input_points[0][pi]));
	}

	std::vector<glm::dmat4x4> mat(input_points.size());
	for (int si = 0; si < input_points.size(); ++si) {
		mat[si] = glm::translate(mat[si], glm::dvec3(input_points[si][0], 0));
	}

	for (int pi = 0; pi < input_points[0].size() - 2; ++pi) {
		// convert the coordinates
		std::vector<glm::dvec2> p0(input_points.size());
		std::vector<glm::dvec2> p1(input_points.size());
		std::vector<glm::dvec2> p2(input_points.size());
		for (int si = 0; si < input_points.size(); ++si) {
			p0[si] = glm::dvec2(glm::inverse(mat[si]) * glm::dvec4(input_points[si][pi], 0, 1));
			p1[si] = glm::dvec2(glm::inverse(mat[si]) * glm::dvec4(input_points[si][pi + 1], 0, 1));
			p2[si] = glm::dvec2(glm::inverse(mat[si]) * glm::dvec4(input_points[si][pi + 2], 0, 1));
		}

		std::vector<glm::dvec2> v(input_points.size());
		for (int si = 0; si < input_points.size(); ++si) {
			v[si] = p1[si] - p2[si];
			v[si] = v[si] / glm::length(v[si]);
		}
		
		double l = 20;
		std::vector<glm::dvec2> pts2(input_points.size());
		for (int si = 0; si < input_points.size(); ++si) {
			pts2[si] = p1[si] + v[si] * l - p0[si];
		}
				
		glm::dvec2 perp = pts2[0] - pts2[1];
		perp = glm::dvec2(-perp.y, perp.x);
		glm::dvec2 c = (pts2[0] + pts2[1]) * 0.5;
		glm::dvec2 prev_pt;
		if (pi == 0) {
			prev_pt = p0[0] + glm::dvec2(100, 0);
		}
		else {
			prev_pt = glm::dvec2(glm::inverse(mat[0]) * glm::dvec4(input_points[0][pi - 1], 0, 1));
		}
		glm::dvec2 pts1 = lineLineIntersection(c, c + perp, glm::dvec2(0, 0), prev_pt - p0[pi]);

		add_lengths.push_back(glm::length(pts1));
		add_lengths.push_back(l);
		add_lengths.push_back(glm::length(pts2[0] - pts1));

		// update the local coordinate systems
		for (int si = 0; si < input_points.size(); ++si) {
			double rot = atan2(p1[si].y, p1[si].x);
			mat[si] = glm::rotate(glm::translate(mat[si], glm::dvec3(p1[si], 0)), rot - M_PI, glm::dvec3(0, 0, 1));
		}
	}
}

void Canvas::forwardKinematics(double theta) {
	if (points.size() == 0) return;

	// calcualte the position of all the points
	points.resize(1);
	points.push_back(points[0] + glm::dvec2(cos(theta), sin(theta)) * lengths[0]);

	add_points.clear();
	for (int i = 1; i < lengths.size(); ++i) {
		double theta = M_PI;
		if (i > 1) {
			glm::dvec2 v = points[i - 1] - points[i - 2];
			theta = atan2(v.y, v.x);
		}
		glm::dvec2 p1 = points[i - 1] + glm::dvec2(cos(theta), sin(theta)) * add_lengths[(i - 1) * 3];
		glm::dvec2 p2 = circleCircleIntersection(points[i], add_lengths[(i - 1) * 3 + 1], p1, add_lengths[(i - 1) * 3 + 2]);
		add_points.push_back(p1);
		add_points.push_back(p2);

		glm::dvec2 v = points[i] - p2;
		v = v / glm::length(v);
		points.push_back(points[i] + v * lengths[i]);
	}
}

void Canvas::stepForward(int step_size) {
	theta += 0.02 * step_size;

	try {
		forwardKinematics(theta);
	}
	catch (char* ex) {
		theta -= 0.02 * step_size;
		forwardKinematics(theta);
	}

	std::cout << theta / M_PI * 180 << std::endl;

	update();
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	if (points.size() == 0) return;

	// draw links
	painter.setPen(QPen(QColor(0, 0, 0), 2));
	for (int i = 0; i < points.size() - 1; ++i) {
		painter.drawLine(points[i].x, height() - points[i].y, points[i + 1].x, height() - points[i + 1].y);
		if (i * 2 + 1 < add_points.size()) {
			painter.drawLine(points[i].x, height() - points[i].y, add_points[i * 2].x, height() - add_points[i * 2].y);
			painter.drawLine(add_points[i * 2].x, height() - add_points[i * 2].y, add_points[i * 2 + 1].x, height() - add_points[i * 2 + 1].y);
			painter.drawLine(points[i + 1].x, height() - points[i + 1].y, add_points[i * 2 + 1].x, height() - add_points[i * 2 + 1].y);
		}
	}

	// draw joints
	painter.setBrush(QBrush(QColor(255, 255, 255)));
	for (int i = 0; i < points.size(); ++i) {
		painter.drawEllipse(QPoint(points[i].x, height() - points[i].y), 5, 5);
	}
	for (int i = 0; i < add_points.size(); ++i) {
		painter.drawEllipse(QPoint(add_points[i].x, height() - add_points[i].y), 5, 5);
	}

	// draw trace
	if (trace.size() > 0) {
		painter.setPen(QPen(QColor(0, 0, 255), 2));
		for (int i = 0; i < trace.size() - 1; ++i) {
			painter.drawLine(trace[i].x, height() - trace[i].y, trace[i + 1].x, trace[i + 1].y);
		}
	}
}

void Canvas::mousePressEvent(QMouseEvent* e) {
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
}

void Canvas::mouseDoubleClickEvent(QMouseEvent* e) {
}

void Canvas::resizeEvent(QResizeEvent *e) {
}

void Canvas::keyPressEvent(QKeyEvent* e) {
	ctrlPressed = false;
	shiftPressed = false;

	if (e->modifiers() & Qt::ControlModifier) {
		ctrlPressed = true;
	}
	if (e->modifiers() & Qt::ShiftModifier) {
		shiftPressed = true;
	}

	switch (e->key()) {
	case Qt::Key_Escape:
		break;
	case Qt::Key_Space:
		break;
	case Qt::Key_Delete:
		break;
	}

	update();
}

void Canvas::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Control:
		ctrlPressed = false;
		break;
	case Qt::Key_Shift:
		shiftPressed = false;
		break;
	default:
		break;
	}
}

