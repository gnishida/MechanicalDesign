#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <QTimer>

glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius2, bool debug = false);

class Link;

class Point {
public:
	int id;
	glm::vec2 pos;
	std::vector<boost::shared_ptr<Link>> out_links;
	std::vector<boost::shared_ptr<Link>> in_links;

public:
	Point(int id, const glm::vec2& pos) : id(id), pos(pos) {}
};

class Link {
public:
	boost::shared_ptr<Point> start_point;
	boost::shared_ptr<Point> end_point;
	float length;

public:
	Link() {}
	Link(boost::shared_ptr<Point> start_point, boost::shared_ptr<Point> end_point);
};

class Gear {
public:
	glm::vec2 center;
	float radius;
	float phase;

public:
	Gear() {}
	Gear(const glm::vec2& center, float radius) : center(center), radius(radius) {}

	glm::vec2 getLinkEndPosition();
	void draw(QPainter& painter);
};

class MechanicalAssembly {
public:
	Gear gear1;
	Gear gear2;
	float link_length1;
	float link_length2;
	float link_length3;
	boost::shared_ptr<Point> marker_point;

public:
	MechanicalAssembly() {}

	glm::vec2 getIntermediateJointPosition();
	glm::vec2 getEndJointPosition();
	void forward(float step);
	void draw(QPainter& painter);
};

class Canvas : public QWidget {
Q_OBJECT

private:
	bool ctrlPressed;
	bool shiftPressed;

	std::vector<boost::shared_ptr<Point>> points;
	std::vector<boost::shared_ptr<Link>> links;
	std::vector<boost::shared_ptr<MechanicalAssembly>> assemblies;
	std::vector<glm::vec2> trace_marker_points;
	QTimer* animation_timer;

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

	void forwardKinematics();
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
