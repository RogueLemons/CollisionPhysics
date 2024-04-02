
#include <iostream>
#include "Engine.h"
#include "Dice.h"

int main()
{
    Engine engine(1000, 800, 3, 3);
    while (engine.isRunning())
    {
        engine.update();
        engine.render();
        engine.display();
    }

    std::cout << "Hello World!" << "\n";
    std::cout << Roll::d(6) << "\n";
}