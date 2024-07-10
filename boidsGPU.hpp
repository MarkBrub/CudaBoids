#pragma once

#include <memory>

#include "simulation.hpp"
#include "boids.hpp"

class BoidsGPU : public Simulation {
	BoidsData* settings = nullptr;
	std::vector<Boid> m_boids;

public:
	std::vector<int> m_ids;
	std::vector<float> m_xPositions;
	std::vector<float> m_yPositions;
	std::vector<float> m_xVelocities;
	std::vector<float> m_yVelocities;

private:

	// GPU memory
	float* m_d_xPositions = nullptr;
	float* m_d_yPositions = nullptr;
	float* m_d_xVelocities = nullptr;
	float* m_d_yVelocities = nullptr;
	float* m_d_xAccelerations = nullptr;
	float* m_d_yAccelerations = nullptr;
	uint32_t* m_d_data = nullptr;

public:
	BoidsGPU(BoidsData& data, size_t width, size_t height);

	~BoidsGPU();

	void initFromCPU(std::shared_ptr<Simulation> cpu);

	void step();
	std::shared_ptr<uint32_t[]> get_data();
	void resize(uint32_t width, uint32_t height);
	void show_settings();

private:
	void changePopulationSize(const int newCount);
};

extern void copySettingsToGPU(int width, int height, float seperation, float alignment, float cohesion, int speedCap, int visionRadius, int avoidanceRadius, float scale);
extern void calculateAcceleration(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids);
extern void updatePosition(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids);