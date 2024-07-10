#pragma once

#include <memory>

#include "simulation.hpp"
#include "boids.hpp"

class BoidsCPU : public Simulation {
	BoidsData* settings = nullptr;

public:
	std::vector<Boid> m_boids;

	BoidsCPU(BoidsData& data, size_t width, size_t height);

	~BoidsCPU() {}

	void initFromGPU(std::shared_ptr<Simulation> gpu);

	void step();
	std::shared_ptr<uint32_t[]> get_data();
	void resize(uint32_t width, uint32_t height);
	void show_settings();

private:
	double m_visionRadiusSquared = settings->m_visionRadius * settings->m_visionRadius;
	double m_avoidanceRadiusSquared = settings->m_avoidanceRadius * settings->m_avoidanceRadius;

	void changePopulationSize(const int newCount);
	Vector2 acceleration(Boid& self);
	void bound(Boid& boid);
};