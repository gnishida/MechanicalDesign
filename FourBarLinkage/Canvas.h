#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
//#include <boost/shared_ptr.hpp>
#include <QTimer>

class Canvas : public QWidget {
Q_OBJECT

public:
	bool ctrlPressed;
	bool shiftPressed;

	std::vector<glm::dvec2> ground_points;
	std::vector<glm::dvec2> points;
	std::vector<glm::dvec2> point_flows;
	std::vector<double> lengths;
	double theta;
	QTimer* animation_timer;
	std::vector<glm::dvec2> trace;
	int selected_point_id;
	double speed;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void forwardKinematics();
	void stepForward();
	void stepBackward();
	void run();
	void stop();
	void open(const QString& filename);
	void save(const QString& filename);

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
