#include "Dice.h"
#include <random>


int Roll::d(int size)
{
    return 1 + fraction() * size;
}

int Roll::fromZeroTo(int max)
{
    return fraction() * (max + 1);
}

int Roll::from_to_(int min, int max)
{
    return min + fromZeroTo(max - min);
}

double Roll::fraction()
{
    static std::mt19937 gen{ std::random_device{}() };
    static std::uniform_int_distribution<> distrib(0, 99);
    return 0.01 * distrib(gen);
}

double Roll::signedFraction()
{
    return 0.01 * from_to_(-100, 100);
}
