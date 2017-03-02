#include "Kinematics.h"
#include <iostream>
#include <QFile>
#include <QDomDocument>
#include <QTextStream>
#include <QDate>

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

	Link::Link(int start, int end, float length) {
		this->start = start;
		this->end = end;
		this->length = length;
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
		glm::vec2 p1 = gears[order.first].getLinkEndPosition();
		glm::vec2 p2 = gears[order.second].getLinkEndPosition();

		return circleCircleIntersection(p1, link_lengths[order.first], p2, link_lengths[order.second]);
	}

	glm::vec2 MechanicalAssembly::getEndEffectorPosition() {
		glm::vec2 p1 = gears[order.first].getLinkEndPosition();
		glm::vec2 p2 = gears[order.second].getLinkEndPosition();

		glm::vec2 joint = circleCircleIntersection(p1, link_lengths[order.first], p2, link_lengths[order.second]);
		glm::vec2 dir = joint - gears[0].getLinkEndPosition();

		return gears[0].getLinkEndPosition() + dir / link_lengths[0] * (link_lengths[0] + link_lengths[2]);
	}

	void MechanicalAssembly::forward(float step) {
		for (int i = 0; i < gears.size(); ++i) {
			gears[i].phase += step * gears[i].speed;
		}
				
		end_effector->pos = getEndEffectorPosition();
	}

	void MechanicalAssembly::draw(QPainter& painter) {
		glm::vec2 p1 = gears[0].getLinkEndPosition();
		glm::vec2 p2 = gears[1].getLinkEndPosition();
		glm::vec2 intP = getIntermediateJointPosition();
		glm::vec2 endP = getEndEffectorPosition();

		// draw gears
		for (int i = 0; i < gears.size(); ++i) {
			gears[i].draw(painter);
		}

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

	void Kinematics::load(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::ReadOnly | QFile::Text)) throw "Fild cannot open.";

		QDomDocument doc;
		doc.setContent(&file);

		QDomElement root = doc.documentElement();
		if (root.tagName() != "design")	throw "Invalid file format.";

		// clear the data
		points.clear();
		assemblies.clear();
		links.clear();
		bodies.clear();
		trace_end_effector.clear();

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
						ass->end_effector = points[end_effector_id];

						QDomNode assembly_part_node = assembly_node.firstChild();
						while (!assembly_part_node.isNull()) {
							if (assembly_part_node.toElement().tagName() == "gear") {
								float x = assembly_part_node.toElement().attribute("x").toFloat();
								float y = assembly_part_node.toElement().attribute("y").toFloat();
								float radius = assembly_part_node.toElement().attribute("radius").toFloat();
								float phase = assembly_part_node.toElement().attribute("phase").toFloat();
								float speed = assembly_part_node.toElement().attribute("speed").toFloat();

								ass->gears.push_back(Gear(glm::vec2(x, y), radius, phase, speed));
							}
							else if (assembly_part_node.toElement().tagName() == "order") {
								int id1 = assembly_part_node.toElement().attribute("id1").toInt();
								int id2 = assembly_part_node.toElement().attribute("id2").toInt();
								ass->order = std::make_pair(id1, id2);
							}
							else if (assembly_part_node.toElement().tagName() == "link") {
								float length = assembly_part_node.toElement().attribute("length").toFloat();
								ass->link_lengths.push_back(length);
							}

							assembly_part_node = assembly_part_node.nextSibling();
						}

						ass->end_effector->pos = ass->getEndEffectorPosition();
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
						boost::shared_ptr<Link> link = boost::shared_ptr<Link>(new Link(start, end, glm::length(points[start]->pos - points[end]->pos)));
						links.push_back(link);

						// set outgoing link to the point
						points[start]->out_links.push_back(link);

						// set incoming link to the point
						if (points[end]->in_links.size() < order + 1) points[end]->in_links.resize(order + 1);
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
						Part part(id1, id2);

						// setup rotation matrix
						glm::vec2 dir = points[id2]->pos - points[id1]->pos;
						float angle = atan2f(dir.y, dir.x);
						glm::vec2 p1 = (points[id1]->pos + points[id2]->pos) * 0.5f;
						glm::mat4x4 model;
						model = glm::rotate(model, -angle, glm::vec3(0, 0, 1));						
												
						QDomNode point_node = body_node.firstChild();
						while (!point_node.isNull()) {
							if (point_node.toElement().tagName() == "point") {
								float x = point_node.toElement().attribute("x").toFloat();
								float y = point_node.toElement().attribute("y").toFloat();

								// convert the coordinates to the local coordinate system
								glm::vec2 rotated_p = glm::vec2(model * glm::vec4(x - p1.x, y - p1.y, 0, 1));
								
								part.points.push_back(rotated_p);
							}

							point_node = point_node.nextSibling();
						}

						bodies.push_back(part);
					}

					body_node = body_node.nextSibling();
				}
			}

			node = node.nextSibling();
		}

		trace_end_effector.resize(assemblies.size());
	}

	void Kinematics::save(const QString& filename) {
		QFile file(filename);
		if (!file.open(QFile::WriteOnly)) throw "File cannot open.";

		QDomDocument doc;

		// set root node
		QDomElement root = doc.createElement("design");
		root.setAttribute("author", "Gen Nishida");
		root.setAttribute("version", "1.0");
		root.setAttribute("date", QDate::currentDate().toString("MM/dd/yyyy"));
		doc.appendChild(root);

		// write points
		QDomElement points_node = doc.createElement("points");
		root.appendChild(points_node);
		for (auto it = points.begin(); it != points.end(); ++it) {
			QDomElement point_node = doc.createElement("point");
			point_node.setAttribute("id", it.key());
			point_node.setAttribute("x", it.value()->pos.x);
			point_node.setAttribute("y", it.value()->pos.y);
			points_node.appendChild(point_node);
		}

		// write assemblies
		QDomElement assemblies_node = doc.createElement("assemblies");
		root.appendChild(assemblies_node);
		for (int i = 0; i < assemblies.size(); ++i) {
			QDomElement assembly_node = doc.createElement("assembly");
			assembly_node.setAttribute("end_effector", assemblies[i]->end_effector->id);

			// write gears
			for (int j = 0; j < assemblies[i]->gears.size(); ++j) {
				QDomElement gear_node = doc.createElement("gear");
				gear_node.setAttribute("x", assemblies[i]->gears[j].center.x);
				gear_node.setAttribute("y", assemblies[i]->gears[j].center.y);
				gear_node.setAttribute("radius", assemblies[i]->gears[j].radius);
				gear_node.setAttribute("phase", assemblies[i]->gears[j].phase);
				gear_node.setAttribute("speed", assemblies[i]->gears[j].speed);
				
				assembly_node.appendChild(gear_node);
			}

			// write order
			QDomElement order_node = doc.createElement("order");
			order_node.setAttribute("id1", assemblies[i]->order.first);
			order_node.setAttribute("id2", assemblies[i]->order.second);
			assembly_node.appendChild(order_node);

			// write links
			for (int j = 0; j < assemblies[i]->link_lengths.size(); ++j) {
				QDomElement link_node = doc.createElement("link");
				link_node.setAttribute("length", assemblies[i]->link_lengths[j]);
				assembly_node.appendChild(link_node);
			}

			assemblies_node.appendChild(assembly_node);
		}

		// write links
		QDomElement links_node = doc.createElement("links");
		root.appendChild(links_node);
		for (auto it = points.begin(); it != points.end(); ++it) {
			for (int j = 0; j < it.value()->in_links.size(); ++j) {
				QDomElement link_node = doc.createElement("link");
				link_node.setAttribute("order", j);
				link_node.setAttribute("start", it.value()->in_links[j]->start);
				link_node.setAttribute("end", it.value()->in_links[j]->end);
				links_node.appendChild(link_node);
			}
		}

		// write bodies
		QDomElement bodies_node = doc.createElement("bodies");
		root.appendChild(bodies_node);
		for (int i = 0; i < bodies.size(); ++i) {
			QDomElement body_node = doc.createElement("body");
			body_node.setAttribute("id1", bodies[i].pivot1);
			body_node.setAttribute("id2", bodies[i].pivot2);
			bodies_node.appendChild(body_node);

			// setup rotation matrix
			glm::vec2 dir = points[bodies[i].pivot2]->pos - points[bodies[i].pivot1]->pos;
			float angle = atan2f(dir.y, dir.x);
			glm::vec2 p1 = (points[bodies[i].pivot1]->pos + points[bodies[i].pivot2]->pos) * 0.5f;
			glm::mat4x4 model;
			model = glm::rotate(model, angle, glm::vec3(0, 0, 1));
			
			for (int k = 0; k < bodies[i].points.size(); ++k) {
				// convert the coordinates to the local coordinate system
				glm::vec2 rotated_p = glm::vec2(model * glm::vec4(bodies[i].points[k].x, bodies[i].points[k].y, 0, 1)) + p1;

				QDomElement point_node = doc.createElement("point");
				point_node.setAttribute("x", rotated_p.x);
				point_node.setAttribute("y", rotated_p.y);
				body_node.appendChild(point_node);
			}
		}

		QTextStream out(&file);
		doc.save(out, 4);
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

				if (point->in_links.size() > 2) throw "forward kinematics error. Overconstrained.";

				// if the parent points are not updated, postpone updating this point
				boost::shared_ptr<Link> l1 = point->in_links[0];
				if (!updated[l1->start]) {
					queue.push_back(point);
					continue;
				}
				if (point->in_links.size() == 2) {
					boost::shared_ptr<Link> l2 = point->in_links[1];
					if (!updated[l2->start]) {
						queue.push_back(point);
						continue;
					}

					// update this point based on two adjacent points
					point->pos = circleCircleIntersection(points[l1->start]->pos, l1->length, points[l2->start]->pos, l2->length);
				}
				else {
					// pin joint
				}
				updated[point->id] = true;
			}
		}
		catch (char* ex) {
			throw "forward kinematics error.";
		}
	}

	void Kinematics::stepForward() {
		for (int i = 0; i < assemblies.size(); ++i) {
			trace_end_effector[i].push_back(assemblies[i]->getEndEffectorPosition());
			assemblies[i]->forward(0.03);
		}

		forwardKinematics();
	}

	void Kinematics::stepBackward() {
		for (int i = 0; i < assemblies.size(); ++i) {
			trace_end_effector[i].push_back(assemblies[i]->getEndEffectorPosition());
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
				glm::vec2 dir = points[bodies[i].pivot2]->pos - points[bodies[i].pivot1]->pos;
				float angle = atan2f(dir.y, dir.x) / M_PI * 180;
				glm::vec2 p1 = (points[bodies[i].pivot1]->pos + points[bodies[i].pivot2]->pos) * 0.5f;
				painter.translate(p1.x, p1.y);
				painter.rotate(angle);
				std::vector<QPointF> points;
				for (int k = 0; k < bodies[i].points.size(); ++k) {
					points.push_back(QPointF(bodies[i].points[k].x, bodies[i].points[k].y));
				}
				painter.drawPolygon(points.data(), points.size());
				//painter.drawEllipse(QPointF(0, 0), glm::length(dir1) * 0.6, glm::length(dir1) * 0.2);
				painter.restore();
			}
		}

		if (show_assemblies) {
			// draw trace
			painter.setPen(QPen(QColor(255, 0, 0), 1));
			for (int i = 0; i < trace_end_effector.size(); ++i) {
				if (trace_end_effector[i].size() > 0) {
					for (int j = std::max(0, (int)trace_end_effector[i].size() - 240); j < trace_end_effector[i].size() - 1; ++j) {
						painter.drawLine(trace_end_effector[i][j].x, trace_end_effector[i][j].y, trace_end_effector[i][j + 1].x, trace_end_effector[i][j + 1].y);
					}
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
				painter.drawLine(points[links[i]->start]->pos.x, points[links[i]->start]->pos.y, points[links[i]->end]->pos.x, points[links[i]->end]->pos.y);
				painter.drawEllipse(QPoint(points[links[i]->start]->pos.x, points[links[i]->start]->pos.y), 3, 3);
				painter.drawEllipse(QPoint(points[links[i]->end]->pos.x, points[links[i]->end]->pos.y), 3, 3);
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
