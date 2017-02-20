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

Canvas::Canvas(QWidget *parent) : QWidget(parent) {
	ctrlPressed = false;
	shiftPressed = false;

	animation_timer = NULL;
	selected_gear = NULL;
	
	//ass->forward(1.5);
	try {
		kinematics.forwardKinematics();
	}
	catch (char* ex) {
		std::cerr << "Initialization error:" << std::endl;
		std::cerr << ex << std::endl;
	}
}

Canvas::~Canvas() {
}

void Canvas::run() {
	if (animation_timer == NULL) {
		animation_timer = new QTimer(this);
		connect(animation_timer, SIGNAL(timeout()), this, SLOT(animation_update()));
		animation_timer->start(10);
	}
}

void Canvas::stop() {
	if (animation_timer != NULL) {
		animation_timer->stop();
		delete animation_timer;
		animation_timer = NULL;
	}
}

void Canvas::showAssemblies(bool flag) {
	kinematics.showAssemblies(flag);
	update();
}

void Canvas::showLinks(bool flag) {
	kinematics.showLinks(flag);
	update();
}

void Canvas::showBodies(bool flag) {
	kinematics.showBodies(flag);
	update();
}

void Canvas::animation_update() {
	try {
		kinematics.stepForward();
	}
	catch (char* ex) {
		//kinematics.stepBackward();
		stop();
		std::cerr << "Animation is stopped by error:" << std::endl;
		std::cerr << ex << std::endl;
	}

	update();
}

void Canvas::paintEvent(QPaintEvent *e) {
	QPainter painter(this);

	kinematics.draw(painter);
}

void Canvas::mousePressEvent(QMouseEvent* e) {
	// hit test against gears
	for (int i = 0; i < kinematics.assemblies.size(); ++i) {
		float dist1 = glm::length(kinematics.assemblies[i]->gear1.center - glm::vec2(e->x(), e->y()));
		if (dist1 <= kinematics.assemblies[i]->gear1.radius) {
			// select this gear
			selected_gear = &kinematics.assemblies[i]->gear1;
			prev_mouse_pt = glm::vec2(e->x(), e->y());
			break;
		}

		float dist2 = glm::length(kinematics.assemblies[i]->gear2.center - glm::vec2(e->x(), e->y()));
		if (dist2 <= kinematics.assemblies[i]->gear2.radius) {
			// select this gear
			selected_gear = &kinematics.assemblies[i]->gear2;
			prev_mouse_pt = glm::vec2(e->x(), e->y());
			break;
		}
	}
}

void Canvas::mouseMoveEvent(QMouseEvent* e) {
	if (selected_gear != NULL) {
		// move this gear
		selected_gear->center += glm::vec2(e->x(), e->y()) - prev_mouse_pt;
		prev_mouse_pt = glm::vec2(e->x(), e->y());
		update();
	}
}

void Canvas::mouseReleaseEvent(QMouseEvent* e) {
	selected_gear = NULL;
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

