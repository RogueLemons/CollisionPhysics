#include "Engine.h"
#include "Dice.h"

// Constructor
Engine::Engine(int windowWidth, int windowHeight, int polygonColumns, int polygonRows)
    : _collisionManager(windowWidth, windowHeight)
{
    initializeWindow(windowWidth, windowHeight);
    initializePolygons(polygonColumns, polygonRows);
}

// Accessors
const bool Engine::isRunning() const
{
    return _window->isOpen();
}

// Public functions

void Engine::pollEvents()
{
    while (_window->pollEvent(_event))
    {
        if (_event.type == sf::Event::Closed)
        {
            _window->close();
        }
        if (_event.type == sf::Event::KeyPressed)
        {
            switch (_event.key.code)
            {
            case sf::Keyboard::Escape:
                _window->close();
                break;
            }
        }
    }

}

void Engine::update()
{
    _dt = _clock.restart().asSeconds();
    pollEvents();
    updatePolygons();
}

void Engine::render()
{
    _window->clear(sf::Color(140, 166, 181));
    renderPolygons();
}

void Engine::display()
{
    _window->display();
}

// Private functions

void Engine::initializeWindow(int width, int height)
{
    _videoMode.width = width;
    _videoMode.height = height;
    int fps = 60;
    _window = std::make_unique<sf::RenderWindow>(_videoMode, "Colliding Polygons 2D");
    _window->setFramerateLimit(fps);
}

void Engine::initializePolygons(int columns, int rows)
{
    // number of polygons = rows * columns
    double k = _videoMode.width * 0.01;
    for (int i = 0; i < columns; i++) {
        for (int j = 0; j < rows; j++) {
            auto p = Polygon(k * Roll::from_to_(5, 8), Roll::from_to_(3, 6));
            auto xPos = _videoMode.width * (i + 1.0) / (columns + 1.0);
            auto yPos = _videoMode.height * (j + 1.0) / (rows + 1.0);
            p.setPosition(xPos, yPos);
            p.setVelocity(k * Roll::from_to_(-10, 10), k * Roll::from_to_(-10, 10), 0);
            _polygons.push_back(p);
        }
    }
}

void Engine::updatePolygons()
{
    _collisionManager.resolveCollisions(_polygons);

    for (auto& polygon : _polygons) {
        _collisionManager.wallCollisionHandling(polygon);
        polygon.updatePosition(_dt);
    }
}

void Engine::renderPolygons()
{
    for (auto& polygon : _polygons) {
        _window->draw(polygon.shape());
    }    
}

