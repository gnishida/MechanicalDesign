#pragma once

#include <vector>
#include <glm/glm.hpp>

class Linkage {
public:
	static enum { CIRCLE_CIRCLE_INTERSECTION_RIGHT = 0, CIRCLE_CIRCLE_INTERSECTION_LEFT };

public:
	std::vector<glm::dvec2> points;
	std::vector<double> lengths;
	int side_of_circle_circle_intersection;

public:
	Linkage();
	~Linkage();
};

