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

#ifndef SQR
#define SQR(x)	((x) * (x))
#endif

glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius2, bool debug) {
	glm::vec2 dir = center2 - center1;
	float d = glm::length(dir);
	if (debug) {
		std::cout << "D: " << d << std::endl;
		std::cout << "radius1: " << radius1 << std::endl;
		std::cout << "radius2: " << radius2 << std::endl;
	}
	//if (d > radius1 + radius2) throw "No intersection";
	if (d > radius1 + radius2) {
		return (center1 + center2) * 0.5f;
	}

	float a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0f;
	float h = sqrtf(radius1 * radius1 - a * a);

	glm::vec2 perp(-dir.y, dir.x);
	perp /= glm::length(perp);

	return center1 + dir * a / d + perp * h;
}

glm::vec2 Gear::getLinkEndPosition() {
	return center + glm::vec2(cos(phase), sin(phase)) * radius;
}

void Gear::draw(QPainter& painter) {
	painter.setPen(QPen(QColor(255, 0, 0), 1));

	int num_split = radius * 0.5;
	for (int i = 0; i < num_split; ++i) {
		float theta1 = i * 3.141592653 * 2.0 / num_split;
		float theta2 = (i + 0.5) * 3.141592653 * 2.0 / num_split;
		float theta3 = (i + 1) * 3.141592653 * 2.0 / num_split;

		glm::vec2 p1 = center + glm::vec2(cos(theta1), sin(theta1)) * (radius + 4);
		glm::vec2 p2 = center + glm::vec2(cos(theta2), sin(theta2)) * (radius + 4);
		glm::vec2 p3 = center + glm::vec2(cos(theta2), sin(theta2)) * radius;
		glm::vec2 p4 = center + glm::vec2(cos(theta3), sin(theta3)) * radius;
		glm::vec2 p5 = center + glm::vec2(cos(theta3), sin(theta3)) * (radius + 4);

		painter.drawLine(p1.x, p1.y, p2.x, p2.y);
		painter.drawLine(p2.x, p2.y, p3.x, p3.y);
		painter.drawLine(p3.x, p3.y, p4.x, p4.y);
		painter.drawLine(p4.x, p4.y, p5.x, p5.y);
	}
}

glm::vec2 MechanicalAssembly::getIntermediateJointPosition() {
	glm::vec2 p1 = gear1.getLinkEndPosition();
	glm::vec2 p2 = gear2.getLinkEndPosition();

	return circleCircleIntersection(p1, link_length1, p2, link_length2);
}

glm::vec2 MechanicalAssembly::getEndJointPosition() {
	glm::vec2 p1 = gear1.getLinkEndPosition();
	glm::vec2 p2 = gear2.getLinkEndPosition();

	glm::vec2 joint = circleCircleIntersection(p1, link_length1, p2, link_length2);
	glm::vec2 dir = joint - p1;

	return p1 + dir / link_length1 * (link_length1 + link_length3);
}

void MechanicalAssembly::forward(float step) {
	gear1.phase -= step;
	gear2.phase -= step;

	marker_point->pos = getEndJointPosition();
}

void MechanicalAssembly::draw(QPainter& painter) {
	glm::vec2 p1 = gear1.getLinkEndPosition();
	glm::vec2 p2 = gear2.getLinkEndPosition();
	glm::vec2 intP = getIntermediateJointPosition();
	glm::vec2 endP = getEndJointPosition();

	// draw gears
	gear1.draw(painter);
	gear2.draw(painter);

	// draw links
	painter.setPen(QPen(QColor(0, 0, 255), 3));
	painter.drawLine(p1.x, p1.y, endP.x, endP.y);
	painter.drawLine(p2.x, p2.y, intP.x, intP.y);
}

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(413, 280))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(512, 298))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(350, 477))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(305, 242))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(287, 453))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(274, 409))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(158, 439))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(166, 474))));
	points.push_back(boost::shared_ptr<Point>(new Point(glm::vec2(230, 571))));

	MechanicalAssembly ass;
	ass.gear1 = Gear(glm::vec2(points[0]->pos.x, points[0]->pos.y), 30);
	ass.gear2 = Gear(glm::vec2(points[1]->pos.x, points[1]->pos.y), 40);
	ass.link_length1 = 120;
	ass.link_length2 = 180;
	ass.link_length3 = 100;
	ass.marker_point = points[2];
	assemblies.push_back(ass);

	ass.marker_point->pos = ass.getEndJointPosition();

	length_p2_p4 = glm::length(points[4]->pos - points[2]->pos);
	length_p3_p4 = glm::length(points[4]->pos - points[3]->pos);
	length_p3_p5 = glm::length(points[5]->pos - points[3]->pos);
	length_p4_p5 = glm::length(points[5]->pos - points[4]->pos);
	length_p5_p6 = glm::length(points[6]->pos - points[5]->pos);
	length_p2_p7 = glm::length(points[7]->pos - points[2]->pos);
	length_p4_p7 = glm::length(points[7]->pos - points[4]->pos);
	length_p6_p7 = glm::length(points[7]->pos - points[6]->pos);
	length_p6_p8 = glm::length(points[8]->pos - points[6]->pos);
	length_p7_p8 = glm::length(points[8]->pos - points[7]->pos);
}

Canvas::~Canvas() {
}

void Canvas::run() {
	animation_timer = new QTimer(this);
	connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
	animation_timer->start(10);
}

void Canvas::stop() {
	animation_timer->stop();
	delete animation_timer;
	animation_timer = NULL;
}

void Canvas::animation_update() {
	for (int i = 0; i < assemblies.size(); ++i) {
		trace_marker_points.push_back(assemblies[i].getEndJointPosition());
		assemblies[i].forward(0.03);
	}

	points[4]->pos = circleCircleIntersection(points[3]->pos, length_p3_p4, points[2]->pos, length_p2_p4);
	points[5]->pos = circleCircleIntersection(points[3]->pos, length_p3_p5, points[4]->pos, length_p4_p5);
	points[7]->pos = circleCircleIntersection(points[4]->pos, length_p4_p7, points[2]->pos, length_p2_p7);
	points[6]->pos = circleCircleIntersection(points[5]->pos, length_p5_p6, points[7]->pos, length_p6_p7);
	points[8]->pos = circleCircleIntersection(points[7]->pos, length_p7_p8, points[6]->pos, length_p6_p8);

	update();
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	// draw trace
	painter.setPen(QPen(QColor(0, 0, 0), 1));
	if (trace_marker_points.size() > 0) {
		for (int i = 0; i < trace_marker_points.size() - 1; ++i) {
			painter.drawLine(trace_marker_points[i].x, trace_marker_points[i].y, trace_marker_points[i + 1].x, trace_marker_points[i + 1].y);
		}
	}

	// draw assembly
	for (int i = 0; i < assemblies.size(); ++i) {
		assemblies[i].draw(painter);
	}

	// draw links
	painter.setPen(QPen(QColor(0, 0, 0), 3));
	painter.drawLine(points[3]->pos.x, points[3]->pos.y, points[4]->pos.x, points[4]->pos.y);
	painter.drawLine(points[3]->pos.x, points[3]->pos.y, points[5]->pos.x, points[5]->pos.y);
	painter.drawLine(points[4]->pos.x, points[4]->pos.y, points[5]->pos.x, points[5]->pos.y);
	painter.drawLine(points[2]->pos.x, points[2]->pos.y, points[4]->pos.x, points[4]->pos.y);
	painter.drawLine(points[2]->pos.x, points[2]->pos.y, points[7]->pos.x, points[7]->pos.y);
	painter.drawLine(points[4]->pos.x, points[4]->pos.y, points[7]->pos.x, points[7]->pos.y);
	painter.drawLine(points[6]->pos.x, points[6]->pos.y, points[7]->pos.x, points[7]->pos.y);
	painter.drawLine(points[7]->pos.x, points[7]->pos.y, points[8]->pos.x, points[8]->pos.y);
	painter.drawLine(points[6]->pos.x, points[6]->pos.y, points[8]->pos.x, points[8]->pos.y);
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

