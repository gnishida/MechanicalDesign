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

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	animation_timer = NULL;

	ground_points.push_back(glm::dvec2(500, 200));
	ground_points.push_back(glm::dvec2(500, 200));
	lengths.push_back(200);
	lengths.push_back(0);
	lengths.push_back(0);
	lengths.push_back(0);
	lengths.push_back(160);

	theta = 140.0 / 180.0 * M_PI;

	std::vector<std::vector<double>> input_thetas(2);
	input_thetas[0].push_back(140 / 180.0 * M_PI);
	input_thetas[0].push_back(50 / 180.0 * M_PI);
	input_thetas[1].push_back(100 / 180.0 * M_PI);
	input_thetas[1].push_back(130 / 180.0 * M_PI);

	std::vector<std::vector<glm::dvec2>> input_points(2);
	for (int i = 0; i < input_points.size(); ++i) {
		input_points[i].push_back(ground_points[0]);
		input_points[i].push_back(ground_points[0] - glm::dvec2(lengths[1], 0));
		input_points[i].push_back(ground_points[0] + glm::dvec2(cos(input_thetas[i][0]), sin(input_thetas[i][0])) * lengths[0]);
		input_points[i].push_back(glm::dvec2());
		double th = input_thetas[i][0] + input_thetas[i][1] - M_PI;
		input_points[i].push_back(input_points[i][2] + glm::dvec2(cos(th), sin(th)) * lengths[4]);
	}

	solveInverse(input_points, input_thetas);
	forwardKinematics(theta);

	/*
	ground_points.push_back(glm::vec2(450, 500));
	ground_points.push_back(glm::vec2(300, 500));
	lengths.push_back(100);
	lengths.push_back(150);
	lengths.push_back(120);
	lengths.push_back(120);
	theta = -M_PI * 0.5;

	forwardKinematics();
	*/
}

Canvas::~Canvas() {
}

void Canvas::solveInverse(std::vector<std::vector<glm::dvec2>>& input_points, std::vector<std::vector<double>>& input_thetas) {
	std::vector<glm::dvec2> v(input_points.size());
	for (int i = 0; i < input_points.size(); ++i) {
		v[i] = input_points[i][2] - input_points[i][4];
		v[i] = v[i] / glm::length(v[i]);
	}

	for (double l = 10; l < 1000; l += 5) {
		lengths[3] = l;
		for (int i = 0; i < input_points.size(); ++i) {
			input_points[i][3] = input_points[i][2] + v[i] * lengths[3];
		}

		glm::dvec2 perp = input_points[0][3] - input_points[1][3];
		perp = glm::dvec2(-perp.y, perp.x);
		perp = perp / glm::length(perp);
		glm::dvec2 c = (input_points[0][3] + input_points[1][3]) * 0.5;
		double t = (ground_points[0].y - c.y) / perp.y;
		for (int i = 0; i < input_points.size(); ++i) {
			input_points[i][1] = c + perp * t;
		}

		ground_points[1] = input_points[0][1];
		lengths[1] = ground_points[0].x - input_points[0][1].x;
		lengths[2] = glm::length(input_points[0][3] - input_points[0][1]);

		try {
			for (int i = 0; i < input_thetas.size(); ++i) {
				forwardKinematics(input_thetas[i][0]);
			}

			std::cout << "L: " << l << std::endl;
			break;
		}
		catch (char* ex) {
		}
	}
}

void Canvas::forwardKinematics(double theta) {
	// calcualte the position of all the points
	points = ground_points;
	points.push_back(points[0] + glm::dvec2(cos(theta), sin(theta)) * lengths[0]);
	points.push_back(circleCircleIntersection(points[2], lengths[3], points[1], lengths[2]));
	glm::dvec2 vec = points[2] - points[3];
	vec = vec / glm::length(vec);
	points.push_back(points[2] + vec * lengths[4]);
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

void Canvas::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
		trace.clear();
	}
}

void Canvas::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void Canvas::animation_update() {
	stepForward(1);
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	if (points.size() < 5) return;

	// draw links
	painter.setPen(QPen(QColor(0, 0, 0), 2));
	painter.drawLine(points[0].x, height() - points[0].y, points[2].x, height() - points[2].y);
	painter.drawLine(points[1].x, height() - points[1].y, points[3].x, height() - points[3].y);
	painter.drawLine(points[2].x, height() - points[2].y, points[3].x, height() - points[3].y);
	painter.drawLine(points[2].x, height() - points[2].y, points[4].x, height() - points[4].y);

	// draw joints
	painter.setBrush(QBrush(QColor(255, 255, 255)));
	for (int i = 0; i < points.size(); ++i) {
		painter.drawEllipse(QPoint(points[i].x, height() - points[i].y), 5, 5);
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

