#include "LinearAlgebra.h"
#include <math.h>

double LinearAlgebra::dot(const Point& a, const Point& b)
{
	return a.x * b.x + a.y * b.y;
}

double LinearAlgebra::cross(const Point& a, const Point& b)
{
	return a.x * b.y - a.y * b.x;
}

LinearAlgebra::Point LinearAlgebra::normal(const Point& a, const Point& b)
{
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double invLength = 1/sqrt(dx * dx + dy * dy);
	return {-dy * invLength, dx * invLength};			// Investigate where - sign comes from
}

LinearAlgebra::Projection LinearAlgebra::project(const std::vector<Point>& polygonCorners, const Point& vector)
{
	Projection proj;
	for (auto& vertex : polygonCorners) {
		auto length = dot(vertex, vector);
		if (length > proj.max)
			proj.max = length;
		if (length < proj.min)
			proj.min = length;
	}
	return proj;
}

bool LinearAlgebra::overlap(const Projection& a, const Projection& b)
{
	return !(a.max < b.min || b.max < a.min);
}
