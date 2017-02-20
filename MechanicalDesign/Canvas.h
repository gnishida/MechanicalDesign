#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include "Kinematics.h"
#include <QTimer>

class Canvas : public QWidget {
Q_OBJECT

private:
	bool ctrlPressed;
	bool shiftPressed;

	kinematics::Kinematics kinematics;
	QTimer* animation_timer;
	kinematics::Gear* selected_gear;
	glm::vec2 prev_mouse_pt;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void run();
	void stop();
	void showAssemblies(bool flag);
	void showLinks(bool flag);
	void showBodies(bool flag);

public slots:
	void animation_update();

protected:
	void paintEvent(QPaintEvent* e);
	void mousePressEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void mouseDoubleClickEvent(QMouseEvent* e);
	void resizeEvent(QResizeEvent *e);

public:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);
};

#endif // CANVAS_H
