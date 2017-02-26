#include "Kinematics.h"
#include <iostream>
#include <QFile>
#include <QDomDocument>

namespace kinematics {
	float M_PI = 3.141592653;

	glm::vec2 circleCircleIntersection(const glm::vec2& center1, float radius1, const glm::vec2& center2, float radius2) {
		glm::vec2 dir = center2 - center1;
		float d = glm::length(dir);
		if (d > radius1 + radius2) {
			throw "No intersection";
		}
		/*if (d > radius1 + radius2) {
			return (center1 * radius2 + center2 + radius1) / (radius1 + radius2);
		}*/

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
		gear1.phase += step;
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

		// draw joints
		painter.setPen(QPen(QColor(0, 0, 255), 3));
		painter.drawEllipse(QPoint(p1.x, p1.y), 3, 3);
		painter.drawEllipse(QPoint(p2.x, p2.y), 3, 3);
		painter.drawEllipse(QPoint(intP.x, intP.y), 3, 3);
		painter.drawEllipse(QPoint(endP.x, endP.y), 3, 3);
	}

	Kinematics::Kinematics() {
		show_assemblies = true;
		show_links = true;
		show_bodies = true;
	}

	bool Kinematics::load(const QString& filename) {
		QFile file(filename);
		file.open(QFile::ReadOnly | QFile::Text);

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	return false;

		// clear the data
		points.clear();
		assemblies.clear();
		links.clear();
		bodies.clear();

		QDomNode node = root.firstChild();
		while (!node.isNull()) {
			if (node.toElement().tagName() == "points") {
				QDomNode point_node = node.firstChild();
				while (!point_node.isNull()) {
					if (point_node.toElement().tagName() == "point") {
						// add a point
						int id = point_node.toElement().attribute("id").toInt();
						float x = point_node.toElement().attribute("x").toFloat();
						float y = point_node.toElement().attribute("y").toFloat();
						points[id] = boost::shared_ptr<Point>(new Point(id, glm::vec2(x, y)));
					}

					point_node = point_node.nextSibling();
				}
			}
			else if (node.toElement().tagName() == "assemblies") {
				QDomNode assembly_node = node.firstChild();
				while (!assembly_node.isNull()) {
					if (assembly_node.toElement().tagName() == "assembly") {
						// add an assembly
						boost::shared_ptr<MechanicalAssembly> ass = boost::shared_ptr<MechanicalAssembly>(new MechanicalAssembly());

						int end_effector_id = assembly_node.toElement().attribute("end_effector").toInt();
						ass->marker_point = points[end_effector_id];

						QDomNode assembly_part_node = assembly_node.firstChild();
						while (!assembly_part_node.isNull()) {
							if (assembly_part_node.toElement().tagName() == "gear1") {
								int center_id = assembly_part_node.toElement().attribute("center").toInt();
								float radius = assembly_part_node.toElement().attribute("radius").toFloat();
								float phase = assembly_part_node.toElement().attribute("phase").toFloat();

								ass->gear1 = Gear(points[center_id]->pos, radius);
								ass->gear1.phase = phase;
							}
							else if (assembly_part_node.toElement().tagName() == "gear2") {
								int center_id = assembly_part_node.toElement().attribute("center").toInt();
								float radius = assembly_part_node.toElement().attribute("radius").toFloat();
								float phase = assembly_part_node.toElement().attribute("phase").toFloat();

								ass->gear2 = Gear(points[center_id]->pos, radius);
								ass->gear2.phase = phase;
							}
							else if (assembly_part_node.toElement().tagName() == "link1") {
								float length = assembly_part_node.toElement().attribute("length").toFloat();
								ass->link_length1 = length;
							}
							else if (assembly_part_node.toElement().tagName() == "link2") {
								float length = assembly_part_node.toElement().attribute("length").toFloat();
								ass->link_length2 = length;
							}
							else if (assembly_part_node.toElement().tagName() == "link3") {
								float length = assembly_part_node.toElement().attribute("length").toFloat();
								ass->link_length3 = length;
							}

							assembly_part_node = assembly_part_node.nextSibling();
						}

						ass->marker_point->pos = ass->getEndJointPosition();
						assemblies.push_back(ass);
					}

					assembly_node = assembly_node.nextSibling();
				}
			}
			else if (node.toElement().tagName() == "links") {
				QDomNode link_node = node.firstChild();
				while (!link_node.isNull()) {
					if (link_node.toElement().tagName() == "link") {
						// add a link
						int order = link_node.toElement().attribute("order").toInt();
						int start = link_node.toElement().attribute("start").toInt();
						int end = link_node.toElement().attribute("end").toInt();
						boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(points[start], points[end]));
						links.push_back(link);

						// set outgoing link to the point
						points[start]->out_links.push_back(link);

						// set incoming link to the point
						if (points[end]->in_links.size() == 0) points[end]->in_links.resize(2);
						points[end]->in_links[order] = link;
					}

					link_node = link_node.nextSibling();
				}
			}
			else if (node.toElement().tagName() == "bodies") {
				QDomNode body_node = node.firstChild();
				while (!body_node.isNull()) {
					if (body_node.toElement().tagName() == "body") {
						// add a body
						int id1 = body_node.toElement().attribute("id1").toInt();
						int id2 = body_node.toElement().attribute("id2").toInt();
						bodies.push_back(std::make_pair(id1, id2));
					}

					body_node = body_node.nextSibling();
				}
			}

			node = node.nextSibling();
		}
	}

	void Kinematics::forwardKinematics() {
		try {
			std::list<boost::shared_ptr<Point>> queue;
			for (auto it = points.begin(); it != points.end(); ++it) {
				queue.push_back(*it);
			}

			std::map<int, bool> updated;
			for (auto it = points.begin(); it != points.end(); ++it) {
				updated[it.key()];
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

	void Kinematics::stepBackward() {
		for (int i = 0; i < assemblies.size(); ++i) {
			trace_marker_points.push_back(assemblies[i]->getEndJointPosition());
			assemblies[i]->forward(-0.03);
		}

		forwardKinematics();
	}

	void Kinematics::draw(QPainter& painter) {
		if (show_bodies) {
			for (int i = 0; i < bodies.size(); ++i) {
				painter.save();
				painter.setPen(QPen(QColor(0, 0, 0), 1));
				painter.setBrush(QBrush(QColor(0, 255, 0, 60)));
				glm::vec2 dir1 = points[bodies[i].second]->pos - points[bodies[i].first]->pos;
				glm::vec2 p1 = (points[bodies[i].first]->pos + points[bodies[i].second]->pos) * 0.5f;
				float ang1 = atan2f(dir1.y, dir1.x) / M_PI * 180;
				painter.translate(p1.x, p1.y);
				painter.rotate(ang1);
				painter.drawEllipse(QPointF(0, 0), glm::length(dir1) * 0.6, glm::length(dir1) * 0.2);
				painter.restore();

			}
		}

		if (show_assemblies) {
			// draw trace
			painter.setPen(QPen(QColor(0, 0, 0), 1));
			if (trace_marker_points.size() > 0) {
				for (int i = std::max(0, (int)trace_marker_points.size() - 240); i < trace_marker_points.size() - 1; ++i) {
					painter.drawLine(trace_marker_points[i].x, trace_marker_points[i].y, trace_marker_points[i + 1].x, trace_marker_points[i + 1].y);
				}
			}

			// draw assembly
			for (int i = 0; i < assemblies.size(); ++i) {
				assemblies[i]->draw(painter);
			}
		}

		if (show_links) {
			// draw links
			painter.setPen(QPen(QColor(0, 0, 0), 3));
			for (int i = 0; i < links.size(); ++i) {
				painter.drawLine(links[i]->start_point->pos.x, links[i]->start_point->pos.y, links[i]->end_point->pos.x, links[i]->end_point->pos.y);
				painter.drawEllipse(QPoint(links[i]->start_point->pos.x, links[i]->start_point->pos.y), 3, 3);
				painter.drawEllipse(QPoint(links[i]->end_point->pos.x, links[i]->end_point->pos.y), 3, 3);
			}
		}
	}

	void Kinematics::showAssemblies(bool flag) {
		show_assemblies = flag;
	}

	void Kinematics::showLinks(bool flag) {
		show_links = flag;
	}

	void Kinematics::showBodies(bool flag) {
		show_bodies = flag;
	}

}
