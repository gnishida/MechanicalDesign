#pragma once

#include <QPainter>
#include <vector>
#include <glm/glm.hpp>
#include <boost/shared_ptr.hpp>
#include <QMap>

namespace kinematics {
	glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius);

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
		int start;
		int end;
		float length;

	public:
		Link() {}
		Link(int start_point, int end_point, float length);
	};

	class Gear {
	public:
		glm::vec2 center;
		float radius;
		float phase;
		float speed;

	public:
		Gear() {}
		Gear(const glm::vec2& center, float radius, float phase, float speed) : center(center), radius(radius), phase(phase), speed(speed) {}

		glm::vec2 getLinkEndPosition();
		void draw(QPainter& painter);
	};

	class MechanicalAssembly {
	public:
		std::vector<Gear> gears;
		std::pair<int, int> order;
		float link_length1;
		float link_length2;
		float link_length3;
		boost::shared_ptr<Point> end_effector;

	public:
		MechanicalAssembly() {}

		glm::vec2 getIntermediateJointPosition();
		glm::vec2 getEndEffectorPosition();
		void forward(float step);
		void draw(QPainter& painter);
	};

	class Kinematics {
	public:
		QMap<int, boost::shared_ptr<Point>> points;
		std::vector<boost::shared_ptr<Link>> links;
		std::vector<boost::shared_ptr<MechanicalAssembly>> assemblies;
		std::vector<std::pair<int, int>> bodies;
		std::vector<std::vector<glm::vec2>> trace_end_effector;

		bool show_assemblies;
		bool show_links;
		bool show_bodies;

	public:
		Kinematics();

		void load(const QString& filename);
		void save(const QString& filename);
		void forwardKinematics();
		void stepForward();
		void stepBackward();
		void draw(QPainter& painter);
		void showAssemblies(bool flag);
		void showLinks(bool flag);
		void showBodies(bool flag);
	};

}
