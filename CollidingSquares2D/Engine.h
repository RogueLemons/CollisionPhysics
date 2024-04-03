#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include "Polygon.h"
#include "CollisionManager.h"

class Engine
{
public:
	// Constructor
	Engine(int windowWidth = 1000, int windowHeight = 800, 
		   int polygonColumns = 5, int polygonRows = 5);

	// Accessors
	const bool isRunning() const;

	// Functions
	void pollEvents();
	void update();
	void render();
	void display();
private:
	// Variables
	std::unique_ptr<sf::RenderWindow> _window;
	sf::VideoMode _videoMode;
	sf::Event _event;
	std::vector<Polygon> _polygons;
	CollisionManager _collisionManager;
	sf::Clock _clock;
	float _dt;

	// Private functions
	void initializeWindow(int width, int height);
	void initializePolygons(int columns, int rows);
	void updatePolygons();
	void renderPolygons();
};