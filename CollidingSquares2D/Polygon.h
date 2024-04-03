#pragma once

#include <vector>
#include <SFML/Graphics.hpp>
#include "LinearAlgebra.h"

class Polygon
{
public:
	//Constructor
	Polygon(double vertexRadius, int nbrOfCorners = 4, double density = 1);
	//Accessors
	const sf::CircleShape& shape() const;
	double mass() const;
	double invInertia() const;
	double vertexRadius() const;
	const std::vector<LinearAlgebra::Point>& vertices();
	double xPos() const;
	double yPos() const;
	double angle() const;
	double xVelocity() const;
	double yVelocity() const;
	double angleVelocity() const;
	bool hasCollidedWith(const Polygon& p) const;
	void addPolygonColission(Polygon&);
	//Functions
	void setVelocity(double x, double y, double a);
	void setPosition(double x, double y);
	void updatePosition(double dt);
private:
	//Variables
	std::vector<LinearAlgebra::Point> _vertices;
	double _vertexRadius;
	int _nbrOfCorners;
	double _mass;
	double _inertia;
	double _inv_inertia;
	sf::CircleShape _shape;
	double _xVel = 0;
	double _yVel = 0;
	double _aVel = 0;
	std::vector<Polygon*> _collisionsThisFrame;
};

