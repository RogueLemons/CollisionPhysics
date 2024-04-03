#pragma once

#include <vector>

namespace LinearAlgebra {

	struct Point {
		double x = 0;
		double y = 0;
	};

	struct Projection {
		double max = std::numeric_limits<double>::lowest();
		double min = std::numeric_limits<double>::max();
	};

	double dot(const Point& a, const Point& b);
	double cross(const Point& a, const Point& b);
	Point normal(const Point& a, const Point& b);
	Projection project(const std::vector<Point>& polygonCorners, const Point& vector);
	bool overlap(const Projection& a, const Projection& b);
}

