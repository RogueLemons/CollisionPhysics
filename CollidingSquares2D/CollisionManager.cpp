#include "CollisionManager.h"
#include <math.h>
using namespace LinearAlgebra;
using std::min;
using std::max;

CollisionManager::CollisionManager(int width2D, int height2D,
	int collisionGridColumns, int collisionGridRows)
{
	_width = width2D;
	_height = height2D;
	_columns = collisionGridColumns;
	_rows = collisionGridRows;
	_collisionGrid.resize(_rows * _columns);
}

void CollisionManager::wallCollisionHandling(Polygon& p) const
{	// Discrete collision
	double C_R = 1.0;

	Point vel = { p.xVelocity(), p.yVelocity() };
	double angleVel = p.angleVelocity();

	auto vertexClosestToX = [&](double X) -> const Point& {
		int closest = 0;
		double distanceToX = std::numeric_limits<double>::max();
		for (int i = 0; i < p.vertices().size(); i++)
		{
			double dx = abs(p.vertices()[i].x - X);
			if (dx < distanceToX)
			{
				closest = i;
				distanceToX = dx;
			}
		}
		return p.vertices()[closest];
	};
	auto vertexClosestToY = [&](double Y) -> const Point& {
		int closest = 0;
		double distanceToY = std::numeric_limits<double>::max();
		for (int i = 0; i < p.vertices().size(); i++)
		{
			double dy = abs(p.vertices()[i].y - Y);
			if (dy < distanceToY)
			{
				closest = i;
				distanceToY = dy;
			}
		}
		return p.vertices()[closest];
	};
	auto calculateNewVelocities = [&](const Point& collision, const Point& normal)
	{
		Point R = { collision.x - p.xPos(), collision.y - p.yPos() };
		double RxN = cross(R, normal);
		Point velTotal = { vel.x - angleVel * R.y, vel.y + angleVel * R.x };
		double impulse = -(1.0 + C_R) * dot(velTotal, normal) / 
						((1.0/p.mass()) + p.invInertia()*RxN*RxN);

		angleVel += p.invInertia() * RxN * impulse;
		vel.x += (impulse / p.mass()) * normal.x;
		vel.y += (impulse / p.mass()) * normal.y;
	};

	const int Big = 10 * _width * _height;
	if (p.xPos() - p.vertexRadius() < 0)
	{
		auto& deepestInWall = vertexClosestToX(-Big);
		if (deepestInWall.x < 0) 
		{
			p.setPosition(p.xPos() - (deepestInWall.x - 0), p.yPos());
			calculateNewVelocities(deepestInWall, { 1, 0 });
		}
	}
	if (p.xPos() + p.vertexRadius() > _width) 
	{
		auto& deepestInWall = vertexClosestToX(Big);
		if (deepestInWall.x > _width)
		{
			p.setPosition(p.xPos() - (deepestInWall.x - _width), p.yPos());
			calculateNewVelocities(deepestInWall, { -1, 0 });
		}
	}
	if (p.yPos() - p.vertexRadius() < 0) 
	{
		auto& deepestInWall = vertexClosestToY(-Big);
		if (deepestInWall.y < 0)
		{
			p.setPosition(p.xPos(), p.yPos() - (deepestInWall.y - 0));
			calculateNewVelocities(deepestInWall, { 0, 1 });
		}
	}
	if (p.yPos() + p.vertexRadius() > _height) 
	{
		auto& deepestInWall = vertexClosestToY(Big);
		if (deepestInWall.y > _height)
		{
			p.setPosition(p.xPos(), p.yPos() - (deepestInWall.y - _height));
			calculateNewVelocities(deepestInWall, { 0, -1 });
		}
	}

	p.setVelocity(vel.x, vel.y, angleVel);
}

void CollisionManager::collisionCheckAndResolution(Polygon& a, Polygon& b) const
{
	if (!rad_collided(a, b))
		return;
	if (!sat_collided(a, b, WithOverlapRemoval))
		return;
	if (a.hasCollidedWith(b))
		return;
	
	auto collision = collisionData(a, b);
	double C_R = 1;		//Coefficient of restitution (1 -> no energy loss)
	if (collision.NormalOnFirstArg) 
	{
		collision.Normal.x *= -1;
		collision.Normal.y *= -1;
	}

	Point a_R = { collision.Point.x - a.xPos(), collision.Point.y - a.yPos() };
	double a_RxN = cross(a_R, collision.Normal);
	Point a_velTotal = { a.xVelocity() - a.angleVelocity() * a_R.y, a.yVelocity() + a.angleVelocity() * a_R.x};
	Point b_R = { collision.Point.x - b.xPos(), collision.Point.y - b.yPos() };
	double b_RxN = cross(b_R, collision.Normal);
	Point b_VelTotal = { b.xVelocity() - b.angleVelocity() * b_R.y, b.yVelocity() + b.angleVelocity() * b_R.x };

	double impulse = -(1.0 + C_R) * dot({ a_velTotal.x - b_VelTotal.x, a_velTotal.y - b_VelTotal.y }, collision.Normal) /
		((1.0 / a.mass()) + (1.0 / b.mass()) + (a.invInertia() * a_RxN * a_RxN + b.invInertia() * b_RxN * b_RxN));

	double a_angleVel = a.angleVelocity() + a.invInertia() * a_RxN * impulse;
	double a_xVel = a.xVelocity() + (impulse / a.mass()) * collision.Normal.x;
	double a_yVel = a.yVelocity() + (impulse / a.mass()) * collision.Normal.y;
	double b_angleVel = b.angleVelocity() - b.invInertia() * b_RxN * impulse;
	double b_xVel = b.xVelocity() - (impulse / b.mass()) * collision.Normal.x;
	double b_yVel = b.yVelocity() - (impulse / b.mass()) * collision.Normal.y;

	a.setVelocity(a_xVel, a_yVel, a_angleVel);
	b.setVelocity(b_xVel, b_yVel, b_angleVel);
	a.addPolygonColission(b);
	b.addPolygonColission(a);
}

void CollisionManager::resolveCollisions(std::vector<Polygon>& polygons)
{	// simple collision optimization, uniform grid space partitioning

	static const auto columnWidth = _width * 1.0 / _columns;
	static const auto rowHeight = _height * 1.0 / _rows;

	// Add polygons to collision grids
	for (auto& polygon : polygons) {
		for (int i = 0; i < _rows; i++) {
			auto top = i * rowHeight;
			auto bottom = (i + 1) * rowHeight;
			bool withinTopToBottom = (polygon.yPos() + polygon.vertexRadius() > top)
				&& (polygon.yPos() - polygon.vertexRadius() < bottom);
			if (!withinTopToBottom)
				continue;

			for (int j = 0; j < _columns; j++) {
				auto left = j * columnWidth;
				auto right = (j + 1) * columnWidth;
				bool withinLeftToRight = (polygon.xPos() + polygon.vertexRadius() > left)
					&& (polygon.xPos() - polygon.vertexRadius() < right);
				if (!withinLeftToRight)
					continue;

				int gridBox = i * _rows + j;
				_collisionGrid[gridBox].push_back(&polygon);
			}
		}
	}

	// Resolve collisions
	for (auto& box : _collisionGrid) {
		gridCollisions(box);
		box.clear();
	}
}

bool CollisionManager::sat_collided(Polygon& a, Polygon& b, SAT_Method handling) const
{	//Seperating Axis Theorem
	double minOverlap = std::numeric_limits<double>::max();

	Point prev = a.vertices().back();
	for (auto& vertex : a.vertices()) {
		auto n = normal(vertex, prev);
		auto aProj = project(a.vertices(), n);
		auto bProj = project(b.vertices(), n);

		if (!overlap(aProj, bProj))
			return false;

		minOverlap = min(min(aProj.max, bProj.max) - max(aProj.min, bProj.min), minOverlap);
		prev = vertex;
	}

	prev = b.vertices().back();
	for (auto& vertex : b.vertices()) {
		auto n = normal(vertex, prev);
		auto aProj = project(a.vertices(), n);
		auto bProj = project(b.vertices(), n);

		if (!overlap(aProj, bProj))
			return false;

		minOverlap = min(min(aProj.max, bProj.max) - max(aProj.min, bProj.min), minOverlap);
		prev = vertex;
	}

	if (handling == WithOverlapRemoval) 
	{
		Point d = { b.xPos() - a.xPos(), b.yPos() - a.yPos() };
		double length = sqrt(d.x * d.x + d.y * d.y);
		d.x /= length;
		d.y /= length;
		
		a.setPosition(a.xPos() - 0.5 * d.x * minOverlap, a.yPos() - 0.5 * d.y * minOverlap);
		b.setPosition(b.xPos() + 0.5 * d.x * minOverlap, b.yPos() + 0.5 * d.y * minOverlap);
	}

	return true;
}

bool CollisionManager::rad_collided(Polygon& a, Polygon& b) const
{
	double dx = a.xPos() - b.xPos();
	double dy = a.yPos() - b.yPos();
	double dSquared = dx * dx + dy * dy;
	double rSum = a.vertexRadius() + b.vertexRadius();
	double rSquared = rSum * rSum;

	if (dSquared > rSquared)
		return false;
	else
		return true;
}

void CollisionManager::gridCollisions(std::vector<Polygon*>& polygons) const
{	// Simple iteration, no optimization
	int size = polygons.size();
	if (size <= 1) return;

	for (int i = 0; i < size - 1; i++) {
		for (int j = i + 1; j < size; j++) {
			collisionCheckAndResolution(*polygons[i], *polygons[j]);
		}
	}
}

void CollisionManager::removeOverlap(Polygon& a, Polygon& b) const
{
	auto dx = a.xPos() - b.xPos();
	auto dy = a.yPos() - b.yPos();
	auto magnitude = sqrt(dx * dx + dy * dy);
	auto depth = a.vertexRadius() + b.vertexRadius() - magnitude;
	auto xPenetration = depth * dx / magnitude;
	auto yPenetration = depth * dy / magnitude;

	a.setPosition(a.xPos() + xPenetration * 0.5, a.yPos() + yPenetration * 0.5);
	b.setPosition(b.xPos() - xPenetration * 0.5, b.yPos() - yPenetration * 0.5);
}

const CollisionData& CollisionManager::collisionData(Polygon& a, Polygon& b) const
{
	auto vertexClosestToOtherCenter = [](Polygon& a, Polygon& b)
	{
		int aIndexDeepest = 0;
		double aVerDisToB = std::numeric_limits<double>::max();
		for (int i = 0; i < a.vertices().size(); i++)
		{
			double dx = a.vertices()[i].x - b.xPos();
			double dy = a.vertices()[i].y - b.yPos();
			double disToB = sqrt(dx * dx + dy * dy);

			if (disToB < aVerDisToB)
			{
				aIndexDeepest = i;
				aVerDisToB = disToB;
			}
		}

		return aIndexDeepest;
	};
	int aIndexDeepest = vertexClosestToOtherCenter(a, b);
	auto& aDeepest = a.vertices()[aIndexDeepest];
	int bIndexDeepest = vertexClosestToOtherCenter(b, a);
	auto& bDeepest = b.vertices()[bIndexDeepest];

	auto unitVector = [](Point a, Point b)
	{
		Point d = { b.x - a.x, b.y - a.y };
		double length = sqrt(d.x * d.x + d.y * d.y);
		d.x /= length;
		d.y /= length;
		return d;
	};
	auto centersVector = unitVector({ a.xPos(), a.yPos() }, { b.xPos(), b.yPos() });
	auto aRelativeVectorOfDeepest = unitVector(aDeepest, { a.xPos(), a.yPos() });
	auto bRelativeVectorOfDeepest = unitVector(bDeepest, { b.xPos(), b.yPos() });

	auto aDepthAlignment = abs(dot(aRelativeVectorOfDeepest, centersVector));
	auto bDepthAlignment = abs(dot(bRelativeVectorOfDeepest, centersVector));

	auto collisionNormal = [](Point aDeepest, Point bDeepest, int bIndexDeepest, Polygon& b) -> Point
	{
		int nextIndex = bIndexDeepest + 1 < b.vertices().size() ? bIndexDeepest + 1 : 0;
		const auto& next = b.vertices()[nextIndex];

		if (min(bDeepest.x, next.x) <= aDeepest.x && aDeepest.x <= max(bDeepest.x, next.x) &&
			min(bDeepest.y, next.y) <= aDeepest.y && aDeepest.y <= max(bDeepest.y, next.y))
		{
			return normal(next, bDeepest);
		}
		else
		{
			int prevIndex = bIndexDeepest - 1 >= 0 ? bIndexDeepest - 1 : b.vertices().size() - 1;
			const auto& prev = b.vertices()[prevIndex];

			return normal(bDeepest, prev);
		}
	};

	if (aDepthAlignment > bDepthAlignment)
		return { aDeepest, collisionNormal(aDeepest, bDeepest, bIndexDeepest, b), false };
	else
		return { bDeepest, collisionNormal(bDeepest, aDeepest, aIndexDeepest, a), true };
}