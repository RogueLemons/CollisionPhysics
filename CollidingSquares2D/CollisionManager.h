#pragma once

#include <vector>
#include "Polygon.h"
#include "LinearAlgebra.h"

enum SAT_Method {
	Detection,
	WithOverlapRemoval
};

struct CollisionData {
	LinearAlgebra::Point Point;
	LinearAlgebra::Point Normal;
	bool NormalOnFirstArg;
};

class CollisionManager
{
public:
	CollisionManager(int width2D, int height2D,
		int collisionGridColumns = 3, int collisionGridRows = 3);
	void wallCollisionHandling(Polygon& p) const;
	void collisionCheckAndResolution(Polygon& a, Polygon& b) const;
	void resolveCollisions(std::vector<Polygon>& polygons);
private:
	int _width;
	int _height;
	int _columns;
	int _rows;
	std::vector<std::vector<Polygon*>> _collisionGrid;
	bool sat_collided(Polygon& a, Polygon& b, SAT_Method handling = Detection) const;
	bool rad_collided(Polygon& a, Polygon& b) const;
	void removeOverlap(Polygon& a, Polygon& b) const;	//Obsolete, for circles only
	void gridCollisions(std::vector<Polygon*>& polygons) const;
	const CollisionData& collisionData(Polygon& a, Polygon& b) const;
};
