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

double crossProduct(const glm::dvec2& vec1, const glm::dvec2& vec2) {
	return vec1.x * vec2.y - vec1.y * vec2.x;
}

glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2) {
	glm::dvec2 dir = center2 - center1;
	double d = glm::length(dir);
	if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
		throw "No intersection";
	}

	double a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0;
	double h = sqrtf(radius1 * radius1 - a * a);

	glm::dvec2 perp(-dir.y, dir.x);
	perp /= glm::length(perp);

	return center1 + dir * a / d + perp * h;
}

glm::dvec2 circleCircleIntersection(const glm::dvec2& center1, double radius1, const glm::dvec2& center2, double radius2, const glm::dvec2& expected_pos) {
	glm::dvec2 dir = center2 - center1;
	double d = glm::length(dir);
	if (d > radius1 + radius2 || d < abs(radius1 - radius2)) {
		throw "No intersection";
	}

	double a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0;
	double h = 0;
	if (radius1 * radius1 > a * a) {
		h = sqrtf(radius1 * radius1 - a * a);
	}

	glm::dvec2 perp(-dir.y, dir.x);
	perp /= glm::length(perp);

	glm::dvec2 pt1 = center1 + dir * a / d + perp * h;
	glm::dvec2 pt2 = center1 + dir * a / d - perp * h;

	if (glm::length(pt1 - expected_pos) < glm::length(pt2 - expected_pos)) {
		return pt1;
	}
	else {
		return pt2;
	}
}

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	animation_timer = NULL;
	selected_point_id = -1;
	speed = 0.02;

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

void Canvas::forwardKinematics() {
	if (ground_points.size() == 0) return;

	std::vector<glm::dvec2> prev_points = points;

	// calcualte the position of all the points
	points = ground_points;
	points.push_back(points[0] + glm::dvec2(cos(theta), sin(theta)) * lengths[0]);

	if (point_flows.size() == 5) {
		points.push_back(circleCircleIntersection(points[2], lengths[2], points[1], lengths[1], prev_points[3] + point_flows[3]));
		points.push_back(circleCircleIntersection(points[2], lengths[3], points[3], lengths[4], prev_points[4] + point_flows[4]));
	}
	else {
		points.push_back(circleCircleIntersection(points[2], lengths[2], points[1], lengths[1]));
		points.push_back(circleCircleIntersection(points[2], lengths[3], points[3], lengths[4]));
	}
}

void Canvas::stepForward() {
	theta += speed;
	std::vector<glm::dvec2> prev_points = points;
	try {
		forwardKinematics();
		point_flows.clear();
		if (prev_points.size() == points.size()) {
			for (int i = 0; i < points.size(); ++i) {
				point_flows.push_back(points[i] - prev_points[i]);
			}
		}
	}
	catch (char* ex) {
		point_flows.clear();
		points = prev_points;
		theta -= speed;
		speed = -speed;
	}

	// update the trace of the end-effector
	if (points.size() >= 5) {
		trace.push_back(points[4]);
		if (trace.size() > 400) trace.erase(trace.begin());
	}

	update();
}

void Canvas::stepBackward() {
	theta -= speed;
	try {
		forwardKinematics();
	}
	catch (char* ex) {
		theta += speed;
		speed = -speed;
	}

	// update the trace of the end-effector
	if (points.size() >= 5) {
		trace.push_back(points[4]);
		if (trace.size() > 400) trace.erase(trace.begin());
	}

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

void Canvas::open(const QString& filename) {
	QFile file(filename);
	if (!file.open(QFile::ReadOnly | QFile::Text)) throw "Fild cannot open.";

	QDomDocument doc;
	doc.setContent(&file);

	QDomElement root = doc.documentElement();
	if (root.tagName() != "design")	throw "Invalid file format.";

	// clear the data
	ground_points.clear();
	lengths.clear();
	trace.clear();
	points.clear();
	point_flows.clear();

	QDomNode node = root.firstChild();
	while (!node.isNull()) {
		if (node.toElement().tagName() == "ground_points") {
			theta = node.toElement().attribute("initial_phase").toDouble();

			QDomNode point_node = node.firstChild();
			while (!point_node.isNull()) {
				if (point_node.toElement().tagName() == "ground_point") {
					// add a ground point
					double x = point_node.toElement().attribute("x").toDouble();
					double y = point_node.toElement().attribute("y").toDouble();
					ground_points.push_back(glm::vec2(x, y));
				}

				point_node = point_node.nextSibling();
			}
		}
		else if (node.toElement().tagName() == "lengths") {
			QDomNode length_node = node.firstChild();
			while (!length_node.isNull()) {
				if (length_node.toElement().tagName() == "length") {
					// add a ground point
					double length = length_node.toElement().attribute("value").toDouble();
					lengths.push_back(length);
				}

				length_node = length_node.nextSibling();
			}
		}

		node = node.nextSibling();
	}

	forwardKinematics();
	update();
}

void Canvas::save(const QString& filename) {
	QFile file(filename);
	if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

	QDomDocument doc;

	// set root node
	QDomElement root = doc.createElement("design");
	root.setAttribute("author", "Gen Nishida");
	root.setAttribute("version", "1.0");
	root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
	doc.appendChild(root);

	// write points
	QDomElement points_node = doc.createElement("ground_points");
	points_node.setAttribute("initial_phase", theta);
	root.appendChild(points_node);
	for (int i = 0; i < ground_points.size(); ++i) {
		QDomElement point_node = doc.createElement("ground_point");
		point_node.setAttribute("x", ground_points[i].x);
		point_node.setAttribute("y", ground_points[i].y);
		points_node.appendChild(point_node);
	}

	// write lengths
	QDomElement lengths_node = doc.createElement("lengths");
	root.appendChild(lengths_node);
	for (int i = 0; i < lengths.size(); ++i) {
		QDomElement length_node = doc.createElement("length");
		length_node.setAttribute("value", lengths[i]);
		lengths_node.appendChild(length_node);
	}

	QTextStream out(&file);
	doc.save(out, 4);
}

void Canvas::animation_update() {
	stepForward();
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	if (points.size() < 5) return;

	// draw links
	painter.setPen(QPen(QColor(0, 0, 0), 2));
	painter.drawLine(points[0].x, points[0].y, points[2].x, points[2].y);
	painter.drawLine(points[1].x, points[1].y, points[3].x, points[3].y);
	painter.drawLine(points[2].x, points[2].y, points[3].x, points[3].y);
	painter.drawLine(points[2].x, points[2].y, points[4].x, points[4].y);
	painter.drawLine(points[3].x, points[3].y, points[4].x, points[4].y);

	// draw joints
	painter.setBrush(QBrush(QColor(255, 255, 255)));
	for (int i = 0; i < points.size(); ++i) {
		if (i == selected_point_id) {
			painter.setPen(QPen(QColor(0, 0, 255), 2));
		}
		else {
			painter.setPen(QPen(QColor(0, 0, 0), 2));
		}
		painter.drawEllipse(QPoint(points[i].x, points[i].y), 5, 5);
	}

	// draw trace
	if (trace.size() > 0) {
		painter.setPen(QPen(QColor(0, 0, 255), 2));
		for (int i = 0; i < trace.size() - 1; ++i) {
			painter.drawLine(trace[i].x, trace[i].y, trace[i + 1].x, trace[i + 1].y);
		}
	}
}

void Canvas::mousePressEvent(QMouseEvent* e) {
	// find the closest point
	double min_dist = 5;
	selected_point_id = -1;
	for (int i = 0; i < points.size(); ++i) {
		double dist = glm::length(glm::dvec2(e->x(), e->y()) - points[i]);
		if (dist < min_dist) {
			min_dist = dist;
			selected_point_id = i;
		}
	}
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
	if (selected_point_id >= 0) {
		std::vector<glm::dvec2> prev_ground_points = ground_points;
		std::vector<glm::dvec2> prev_points = points;
		std::vector<double> prev_lengths = lengths;

		if (selected_point_id == 0) {
			ground_points[0] = glm::dvec2(e->x(), e->y());
			lengths[0] = glm::length(points[2] - ground_points[0]);
			theta = atan2(points[2].y - e->y(), points[2].x - e->x());
		}
		else if (selected_point_id == 1) {
			ground_points[1] = glm::dvec2(e->x(), e->y());
			lengths[1] = glm::length(points[3] - ground_points[1]);
		}
		else if (selected_point_id == 2) {
			lengths[0] = glm::length(glm::dvec2(e->x(), e->y()) - points[0]);
			lengths[2] = glm::length(glm::dvec2(e->x(), e->y()) - points[3]);
			lengths[3] = glm::length(glm::dvec2(e->x(), e->y()) - points[4]);
			theta = atan2(e->y() - points[0].y, e->x() - points[0].x);
		}
		else if (selected_point_id == 3) {
			lengths[1] = glm::length(glm::dvec2(e->x(), e->y()) - points[1]);
			lengths[2] = glm::length(glm::dvec2(e->x(), e->y()) - points[2]);
			lengths[4] = glm::length(glm::dvec2(e->x(), e->y()) - points[4]);
		}
		else if (selected_point_id == 4) {
			lengths[3] = glm::length(glm::dvec2(e->x(), e->y()) - points[2]);
			lengths[4] = glm::length(glm::dvec2(e->x(), e->y()) - points[3]);
		}

		try {
			forwardKinematics();
		}
		catch (char* ex) {
			ground_points = prev_ground_points;
			points = prev_points;
			lengths = prev_lengths;
		}
		update();
	}
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
	selected_point_id = -1;
	update();
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

