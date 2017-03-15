#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <QTimer>
#include "Linkage.h"

class Canvas : public QWidget {
Q_OBJECT

public:
	bool ctrlPressed;
	bool shiftPressed;

	int sketch_seq_no;
	int selected_point_id;

	std::vector<std::vector<glm::dvec2>> input_points;
	std::vector<glm::dvec2> points;
	std::vector<double> lengths;
	std::vector<Linkage> linkages;
	double theta;
	int idx_driving_point;
	std::vector<glm::dvec2> trace;
	double speed;
	std::pair<double, double> angle_range;

	QTimer* animation_timer;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void init();
	void solveInverse(std::vector<std::vector<glm::dvec2>>& input_points, double l);
	void forwardKinematics(double theta);
	void stepForward(int step_size);
	void run();
	void stop();

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
