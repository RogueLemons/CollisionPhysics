#include "Polygon.h"
#include "Dice.h"
#define _USE_MATH_DEFINES
#include <math.h>

using LinearAlgebra::Point;

Polygon::Polygon(double vertexRadius, int nbrOfCorners, double density)
{
	_vertexRadius = vertexRadius;
	_nbrOfCorners = nbrOfCorners;

	double apothem = _vertexRadius * cos(M_PI / _nbrOfCorners);
	double area = apothem * apothem * _nbrOfCorners * tan(M_PI / _nbrOfCorners) * 0.5;
	_mass = density * area;

	_inertia = (_mass * _vertexRadius * _vertexRadius / 6) *
		(sin(M_PI / _nbrOfCorners) * sin(M_PI / _nbrOfCorners) + 3 * cos(M_PI / _nbrOfCorners) * cos(M_PI / _nbrOfCorners));
	_inv_inertia = 1 / _inertia;

	_vertices = std::vector<Point>(_nbrOfCorners);
	_shape = sf::CircleShape(_vertexRadius, _nbrOfCorners);
	_shape.setOrigin(_vertexRadius, _vertexRadius);

	int r = Roll::fromZeroTo(255);
	int g = Roll::fromZeroTo(255);
	int b = Roll::fromZeroTo(255);
	_shape.setFillColor(sf::Color(r, g, b));
}

const sf::CircleShape& Polygon::shape() const
{
	return _shape;
}

double Polygon::mass() const
{
	return _mass;
}

double Polygon::invInertia() const
{
	return _inv_inertia;
}

double Polygon::vertexRadius() const
{
	return _vertexRadius;
}

const std::vector<LinearAlgebra::Point>& Polygon::vertices()
{
	return _vertices;
}

double Polygon::xPos() const
{
	return _shape.getPosition().x;
}

double Polygon::yPos() const
{
	return _shape.getPosition().y;
}

double Polygon::angle() const
{
	return _shape.getRotation() * M_PI / 180;
}

double Polygon::xVelocity() const
{
	return _xVel;
}

double Polygon::yVelocity() const
{
	return _yVel;
}

double Polygon::angleVelocity() const
{
	return _aVel;
}

bool Polygon::hasCollidedWith(const Polygon& p) const
{
	for (auto& polygon : _collisionsThisFrame) 
	{
		if (polygon == &p)
			return true;
	}
	return false;
}

void Polygon::addPolygonColission(Polygon& p)
{
	_collisionsThisFrame.push_back(&p);
}

void Polygon::setVelocity(double x, double y, double a)
{
	_xVel = x;
	_yVel = y;
	_aVel = a;
}

void Polygon::setPosition(double x, double y)
{
	_shape.setPosition(x, y);
}

void Polygon::updatePosition(double dt)
{
	// Members
	double dx = _xVel * dt;
	double dy = _yVel * dt;
	double da = _aVel * dt;

	// Visuals
	_shape.move(dx, dy);
	_shape.rotate(da * 180 / M_PI);

	// Corners
	int i = 0;
	double delta_angle = 2 * M_PI / _nbrOfCorners;
	for (auto& vertex : _vertices) {
		vertex.x = xPos() + _vertexRadius * sin(angle() + i * delta_angle);
		vertex.y = yPos() + _vertexRadius * -cos(angle() + i * delta_angle);
		i++;
	}

	// Reset collisions
	_collisionsThisFrame.clear();
}
