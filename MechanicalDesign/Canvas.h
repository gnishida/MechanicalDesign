#ifndef CANVAS_H
#define CANVAS_H

#include <QWidget>
#include <QKeyEvent>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <QTimer>

glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius2, bool debug = false);

class Point {
public:
	glm::vec2 pos;

public:
	Point(const glm::vec2& pos) : pos(pos) {}
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
	std::vector<MechanicalAssembly> assemblies;
	std::vector<glm::vec2> trace_marker_points;
	QTimer* animation_timer;
	float length_p2_p4;
	float length_p3_p4;
	float length_p3_p5;
	float length_p4_p5;
	float length_p5_p6;
	float length_p2_p7;
	float length_p4_p7;
	float length_p6_p7;
	float length_p6_p8;
	float length_p7_p8;
	/*
	std::vector<boost::shared_ptr<RigidObject>> selectedRigidObjects;
	boost::shared_ptr<RigidObject> rigidObject;
	std::vector<Joint> joints;
	*/

public:
	Canvas(QWidget *parent = NULL);
    ~Canvas();

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
