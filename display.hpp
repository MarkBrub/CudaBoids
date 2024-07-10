#pragma once

#include <iostream>
#include <string>
#include <cmath>

#include "boid.hpp"
#include "vector2.hpp"

class Renderer {
	int totalFrames = 0;

public:
	Renderer(const int width, const int height);
	
	void frameStart();
	void drawBoids(std::vector<Boid>& boids);
};