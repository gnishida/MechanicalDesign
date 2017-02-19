#include "Kinematics.h"
#include <iostream>

namespace kinematics {
	float M_PI = 3.141592653;

	glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius2) {
		glm::vec2 dir = center2 - center1;
		float d = glm::length(dir);
		if (d > radius1 + radius2) {
			throw "No intersection";
		}

		float a = (radius1 * radius1 - radius2 * radius2 + d * d) / d / 2.0f;
		float h = sqrtf(radius1 * radius1 - a * a);

		glm::vec2 perp(-dir.y, dir.x);
		perp /= glm::length(perp);

		return center1 + dir * a / d + perp * h;
	}

	Link::Link(boost::shared_ptr<Point> start_point, boost::shared_ptr<Point> end_point) {
		this->start_point = start_point;
		this->end_point = end_point;
		length = glm::length(end_point->pos - start_point->pos);
	}

	glm::vec2 Gear::getLinkEndPosition() {
		return center + glm::vec2(cos(phase), sin(phase)) * radius;
	}

	/**
	 * Draw a gear.
	 */
	void Gear::draw(QPainter& painter) {
		painter.setPen(QPen(QColor(255, 0, 0), 1));

		painter.drawEllipse(QPoint(center.x, center.y), 4, 4);

		int num_split = radius * 0.4;
		for (int i = 0; i < num_split; ++i) {
			float theta1 = i * M_PI * 2.0 / num_split;
			float theta2 = (i + 0.5) * M_PI * 2.0 / num_split;
			float theta3 = (i + 1) * M_PI * 2.0 / num_split;

			glm::vec2 p1 = center + glm::vec2(cos(phase + theta1), sin(phase + theta1)) * (radius + 5);
			glm::vec2 p2 = center + glm::vec2(cos(phase + theta2), sin(phase + theta2)) * (radius + 5);
			glm::vec2 p3 = center + glm::vec2(cos(phase + theta2), sin(phase + theta2)) * radius;
			glm::vec2 p4 = center + glm::vec2(cos(phase + theta3), sin(phase + theta3)) * radius;
			glm::vec2 p5 = center + glm::vec2(cos(phase + theta3), sin(phase + theta3)) * (radius + 5);

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

	Kinematics::Kinematics() {
		// add points
		points.push_back(boost::shared_ptr<Point>(new Point(0, glm::vec2(413, 280))));
		points.push_back(boost::shared_ptr<Point>(new Point(1, glm::vec2(512, 298))));
		points.push_back(boost::shared_ptr<Point>(new Point(2, glm::vec2(350, 477))));
		points.push_back(boost::shared_ptr<Point>(new Point(3, glm::vec2(305, 242))));
		points.push_back(boost::shared_ptr<Point>(new Point(4, glm::vec2(287, 453))));
		points.push_back(boost::shared_ptr<Point>(new Point(5, glm::vec2(274, 409))));
		points.push_back(boost::shared_ptr<Point>(new Point(6, glm::vec2(158, 439))));
		points.push_back(boost::shared_ptr<Point>(new Point(7, glm::vec2(166, 474))));
		points.push_back(boost::shared_ptr<Point>(new Point(8, glm::vec2(230, 571))));

		// setup assembly
		boost::shared_ptr<MechanicalAssembly> ass = boost::shared_ptr<MechanicalAssembly>(new MechanicalAssembly());
		ass->gear1 = Gear(points[0]->pos, 30);
		ass->gear2 = Gear(points[1]->pos, 40);
		ass->link_length1 = 120;
		ass->link_length2 = 180;
		ass->link_length3 = 100;
		ass->marker_point = points[2];
		assemblies.push_back(ass);
		ass->marker_point->pos = ass->getEndJointPosition();

		// add links
		boost::shared_ptr<Link> link_2_4 = boost::shared_ptr<Link>(new Link(points[2], points[4]));
		boost::shared_ptr<Link> link_3_4 = boost::shared_ptr<Link>(new Link(points[3], points[4]));
		boost::shared_ptr<Link> link_3_5 = boost::shared_ptr<Link>(new Link(points[3], points[5]));
		boost::shared_ptr<Link> link_4_5 = boost::shared_ptr<Link>(new Link(points[4], points[5]));
		boost::shared_ptr<Link> link_5_6 = boost::shared_ptr<Link>(new Link(points[5], points[6]));
		boost::shared_ptr<Link> link_2_7 = boost::shared_ptr<Link>(new Link(points[2], points[7]));
		boost::shared_ptr<Link> link_4_7 = boost::shared_ptr<Link>(new Link(points[4], points[7]));
		boost::shared_ptr<Link> link_6_8 = boost::shared_ptr<Link>(new Link(points[6], points[8]));
		boost::shared_ptr<Link> link_7_6 = boost::shared_ptr<Link>(new Link(points[7], points[6]));
		boost::shared_ptr<Link> link_7_8 = boost::shared_ptr<Link>(new Link(points[7], points[8]));
		links.push_back(link_2_4);
		links.push_back(link_3_4);
		links.push_back(link_3_5);
		links.push_back(link_4_5);
		links.push_back(link_5_6);
		links.push_back(link_2_7);
		links.push_back(link_4_7);
		links.push_back(link_6_8);
		links.push_back(link_7_6);
		links.push_back(link_7_8);

		// set outgoing links to the points
		points[2]->out_links.push_back(link_2_4);
		points[2]->out_links.push_back(link_2_7);
		points[3]->out_links.push_back(link_3_4);
		points[3]->out_links.push_back(link_3_5);
		points[4]->out_links.push_back(link_4_5);
		points[4]->out_links.push_back(link_4_7);
		points[5]->out_links.push_back(link_5_6);
		points[6]->out_links.push_back(link_6_8);
		points[7]->out_links.push_back(link_7_6);
		points[7]->out_links.push_back(link_7_8);

		// set incoming links to the points
		points[4]->in_links.push_back(link_3_4);
		points[4]->in_links.push_back(link_2_4);
		points[5]->in_links.push_back(link_3_5);
		points[5]->in_links.push_back(link_4_5);
		points[6]->in_links.push_back(link_5_6);
		points[6]->in_links.push_back(link_7_6);
		points[7]->in_links.push_back(link_4_7);
		points[7]->in_links.push_back(link_2_7);
		points[8]->in_links.push_back(link_7_8);
		points[8]->in_links.push_back(link_6_8);

	}

	void Kinematics::forwardKinematics() {
		try {
			std::list<boost::shared_ptr<Point>> queue;
			for (int i = 0; i < points.size(); ++i) {
				queue.push_back(points[i]);
			}

			std::map<int, bool> updated;
			for (int i = 0; i < points.size(); ++i) {
				updated[i] = false;
			}

			while (!queue.empty()) {
				boost::shared_ptr<Point> point = queue.front();
				queue.pop_front();

				// if no parent points, set this point as updated.
				if (point->in_links.size() == 0) {
					updated[point->id] = true;
					continue;
				}

				if (point->in_links.size() != 2) throw "forward kinematics error. Overconstrained.";

				// if the parent points are not updated, postpoine updating this point
				boost::shared_ptr<Link> l1 = point->in_links[0];
				if (!updated[l1->start_point->id]) {
					queue.push_back(point);
					continue;
				}
				boost::shared_ptr<Link> l2 = point->in_links[1];
				if (!updated[l2->start_point->id]) {
					queue.push_back(point);
					continue;
				}

				// update this point based on two adjacent points
				point->pos = circleCircleIntersection(l1->start_point->pos, l1->length, l2->start_point->pos, l2->length);
				updated[point->id] = true;
			}
		}
		catch (char* ex) {
			throw "forward kinematics error.";
		}
	}

	void Kinematics::stepForward() {
		for (int i = 0; i < assemblies.size(); ++i) {
			trace_marker_points.push_back(assemblies[i]->getEndJointPosition());
			assemblies[i]->forward(0.03);
		}

		forwardKinematics();
	}

	void Kinematics::draw(QPainter& painter) {
		// draw trace
		painter.setPen(QPen(QColor(0, 0, 0), 1));
		if (trace_marker_points.size() > 0) {
			for (int i = 0; i < trace_marker_points.size() - 1; ++i) {
				painter.drawLine(trace_marker_points[i].x, trace_marker_points[i].y, trace_marker_points[i + 1].x, trace_marker_points[i + 1].y);
			}
		}

		// draw assembly
		for (int i = 0; i < assemblies.size(); ++i) {
			assemblies[i]->draw(painter);
		}

		// draw links
		painter.setPen(QPen(QColor(0, 0, 0), 3));
		for (int i = 0; i < links.size(); ++i) {
			painter.drawLine(links[i]->start_point->pos.x, links[i]->start_point->pos.y, links[i]->end_point->pos.x, links[i]->end_point->pos.y);
		}
	}

}
