#pragma once

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <imgui.h>

#include <chrono>
#include <iostream>

#include "boid.hpp"
#include "simulation.hpp"
#include "util.hpp"
#include "vector2.hpp"

struct BoidsData {
	int m_numBoids = 1000;
	// float m_seperation = .75;
	// float m_alignment = .25;
	// float m_cohesion = .0025;
	float m_seperation = .5f;
	float m_alignment = .05f;
	float m_cohesion = .005f;
	int m_speedCap = 3;
	int m_visionRadius = 50;
	int m_avoidanceRadius = 2;
	float m_scale = .25f;

	// Max values
	int m_maxBoids = 40000;
	float m_maxSeperation = 1.0f;
	float m_maxAlignment = 1.0f;
	float m_maxCohesion = .005f;
	int m_maxSpeedCap = 10;
	int m_maxVisionRadius = 100;
	int m_maxAvoidanceRadius = 15;
	float m_maxScale = 2.0f;

	uint32_t m_BackgroundColor = Util::toUint32({44, 57, 75});
	const std::vector<std::vector<uint32_t>> m_Colors = {{255, 255, 255}, {141, 169, 196}, {220, 133, 31}, {129, 244, 149}};

	bool changed = false;
};


class Boids : public Simulation {
	std::shared_ptr<Simulation> m_boids = nullptr;
	BoidsData m_data;
	bool m_useGPU = false;

	// render durration
	double m_renderTime = 0;
	double m_stepTime = 0;

public:
	Boids();

	void step() override;
	std::shared_ptr<uint32_t[]> get_data() override;
	void resize(uint32_t width, uint32_t height) override;
	void show_settings() override;
};



/*
class Boids : public Simulation {
public:
	Boids();
	Boids(uint32_t width, uint32_t height);
	~Boids();

	void step() override;
	std::shared_ptr<uint32_t[]> get_data() override;
	void resize(uint32_t width, uint32_t height) override;
	void show_settings() override;

private:
	bool m_useGPU = false;

	// Used for CPU based simulation
	std::vector<Boid> m_boids;

	// Used for GPU based simulation
	std::vector<int> m_ids;
	std::vector<float> m_xPositions;
	std::vector<float> m_yPositions;
	std::vector<float> m_xVelocities;
	std::vector<float> m_yVelocities;

	// GPU memory
	float* m_d_xPositions = nullptr;
	float* m_d_yPositions = nullptr;
	float* m_d_xVelocities = nullptr;
	float* m_d_yVelocities = nullptr;
	float* m_d_xAccelerations = nullptr;
	float* m_d_yAccelerations = nullptr;
	uint32_t* m_d_data = nullptr;


	// settings
	int m_numBoids = 1000;
	// float m_seperation = .75;
	// float m_alignment = .25;
	// float m_cohesion = .0025;
	float m_seperation = .05;
	float m_alignment = .05;
	float m_cohesion = .0005;
	int m_speedCap = 5;
	int m_visionRadius = 20;
	int m_avoidanceRadius = 2;
	float m_scale = .5;

	// Max values
	int m_maxBoids = 100000;
	float m_maxSeperation = 1;
	float m_maxAlignment = 1;
	float m_maxCohesion = .005;
	int m_maxSpeedCap = 10;
	int m_maxVisionRadius = 100;
	int m_maxAvoidanceRadius = 25;
	float m_maxScale = 2;

	uint32_t m_BackgroundColor = Util::toUint32({44, 57, 75});
	const std::vector<std::vector<uint32_t>> m_Colors = {{255, 255, 255}, {141, 169, 196}, {220, 133, 31}, {129, 244, 149}};

	// Used for caching
	int m_VisionRadiusSquared = m_visionRadius * m_visionRadius;
	int m_AvoidanceRadiusSquared = m_avoidanceRadius * m_avoidanceRadius;

	void swapDevice();
	void changePopulationSize(const int newCount);

	// CPU based simulation
	void stepCPU();
	void renderCPU();
	Vector2 accelerationCPU(Boid& self);
	void boundCPU(Boid& boid);

	// GPU based simulation
	void stepGPU();
	void renderGPU();
};

extern void copySettingsToGPU(int width, int height, float seperation, float alignment, float cohesion, int speedCap, int visionRadius, int avoidanceRadius, float scale);
extern void calculateAcceleration(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids);
extern void updatePosition(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids);
*/