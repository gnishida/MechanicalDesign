#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <QTimer>

class Canvas : public QWidget {
Q_OBJECT

public:
	bool ctrlPressed;
	bool shiftPressed;

	std::vector<glm::dvec2> ground_points;
	std::vector<glm::dvec2> points;
	std::vector<glm::dvec2> add_points;
	std::vector<double> lengths;
	std::vector<double> add_lengths;
	std::vector<std::vector<double>> thetas;
	double theta;
	QTimer* animation_timer;
	std::vector<glm::dvec2> trace;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void solveInverse(std::vector<std::vector<glm::dvec2>>& input_points);
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
