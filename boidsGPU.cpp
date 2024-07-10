#include "boidsGPU.hpp"
#include "boidsCPU.hpp"

BoidsGPU::BoidsGPU(BoidsData& data, size_t width, size_t height) : settings(&data) {
	m_width = width;
	m_height = height;

	cudaMalloc(&m_d_xPositions, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yPositions, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_xVelocities, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yVelocities, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_xAccelerations, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yAccelerations, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_data, m_width * m_height * sizeof(uint32_t));

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);

	copySettingsToGPU(m_width, m_height,
		settings->m_seperation, settings->m_alignment, settings->m_cohesion, settings->m_speedCap,
		settings->m_visionRadius, settings->m_avoidanceRadius, settings->m_scale);
}

BoidsGPU::~BoidsGPU() {
	cudaFree(m_d_data);
	cudaFree(m_d_xPositions);
	cudaFree(m_d_yPositions);
	cudaFree(m_d_xVelocities);
	cudaFree(m_d_yVelocities);
	cudaFree(m_d_xAccelerations);
	cudaFree(m_d_yAccelerations);
}

void BoidsGPU::initFromCPU(std::shared_ptr<Simulation> sim) {
	// cast sim to BoidsCPU
	auto boidsCPU = std::dynamic_pointer_cast<BoidsCPU>(sim);

	m_boids.resize(settings->m_numBoids);

	m_ids.resize(settings->m_numBoids);
	m_xPositions.resize(settings->m_numBoids);
	m_yPositions.resize(settings->m_numBoids);
	m_xVelocities.resize(settings->m_numBoids);
	m_yVelocities.resize(settings->m_numBoids);

	for (size_t x = 0; x < settings->m_numBoids; x++) {
		m_ids[x] = boidsCPU->m_boids[x].id;
		m_xPositions[x] = boidsCPU->m_boids[x].position.x;
		m_yPositions[x] = boidsCPU->m_boids[x].position.y;
		m_xVelocities[x] = boidsCPU->m_boids[x].velocity.x;
		m_yVelocities[x] = boidsCPU->m_boids[x].velocity.y;
	}
}

void BoidsGPU::step() {
	//settings->m_numBoids = 20000;
	if (m_ids.size() != settings->m_numBoids) {
		changePopulationSize(settings->m_numBoids);
	}

 	cudaMemcpy(m_d_xPositions, m_xPositions.data(), settings->m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_yPositions, m_yPositions.data(), settings->m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_xVelocities, m_xVelocities.data(), settings->m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_yVelocities, m_yVelocities.data(), settings->m_numBoids * sizeof(float), cudaMemcpyHostToDevice);

	calculateAcceleration(m_d_xPositions, m_d_yPositions, m_d_xVelocities, m_d_yVelocities, m_d_xAccelerations, m_d_yAccelerations, settings->m_numBoids);
	updatePosition(m_d_xPositions, m_d_yPositions, m_d_xVelocities, m_d_yVelocities, m_d_xAccelerations, m_d_yAccelerations, settings->m_numBoids);
}

std::shared_ptr<uint32_t[]> BoidsGPU::get_data() {
	//copy data back to CPU
	cudaMemcpy(m_xPositions.data(), m_d_xPositions, settings->m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_yPositions.data(), m_d_yPositions, settings->m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_xVelocities.data(), m_d_xVelocities, settings->m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_yVelocities.data(), m_d_yVelocities, settings->m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);

	// move data back to boids
	// temp to get rendering working
	for (size_t x = 0; x < settings->m_numBoids; x++) {
		m_boids[x].position.x = m_xPositions[x];
		m_boids[x].position.y = m_yPositions[x];
		m_boids[x].velocity.x = m_xVelocities[x];
		m_boids[x].velocity.y = m_yVelocities[x];
	}

	std::fill(m_buffer.get(), m_buffer.get() + m_width * m_height, settings->m_BackgroundColor);
	
	int id = 0;

	for (const auto& boid : m_boids) {
		Vector2 top;
		Vector2 left;
		Vector2 right;

		boid.GetBoidShape(top, left, right, settings->m_scale);

		auto xPair = std::minmax({top.x, left.x, right.x});
		auto yPair = std::minmax({top.y, left.y, right.y});

		int32_t position = 0;
		int32_t maxPosition = m_width * m_height;

		for (int y = yPair.first; y < yPair.second; y++) {
			for (int x = xPair.first; x < xPair.second; x++) {
				// bounds check
				if (x < 0 || x >= m_width || y < 0 || y >= m_height) continue;
				if (!inTriangle(Vector2(x, y), top, left, right)) continue;

				position = x + (m_width * y);
				m_buffer[position] = Util::toUint32(settings->m_Colors[id % 4]);
			}
		}

		id++;
	}

	return m_buffer;
}

void BoidsGPU::resize(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);

	cudaFree(m_d_data);
	cudaMalloc(&m_d_data, m_width * m_height * sizeof(uint32_t));

	copySettingsToGPU(m_width, m_height,
		settings->m_seperation, settings->m_alignment, settings->m_cohesion, settings->m_speedCap,
		settings->m_visionRadius, settings->m_avoidanceRadius, settings->m_scale);
}

void BoidsGPU::show_settings() {
	if (settings->changed) {
		copySettingsToGPU(m_width, m_height,
			settings->m_seperation, settings->m_alignment, settings->m_cohesion, settings->m_speedCap,
			settings->m_visionRadius, settings->m_avoidanceRadius, settings->m_scale);
	}
}

void BoidsGPU::changePopulationSize(const int newCount) {
	if (newCount > m_boids.size()) {
		for (int x = m_boids.size(); x < newCount; x++) {
			m_boids.emplace_back();

			m_ids.emplace_back(x);
			m_xPositions.emplace_back(Util::getRandomNumber(m_width * .1f, m_width * .9f));
			m_yPositions.emplace_back(Util::getRandomNumber(m_height * .1f, m_height * .9f));

			m_xVelocities.emplace_back(Util::getRandomNumber(-3, 3));
			m_yVelocities.emplace_back(Util::getRandomNumber(-3, 3));
		}

	} else {
		// Remove boids from CPU simulation
		m_boids.erase(m_boids.begin() + newCount, m_boids.end());

		// Remove boids from GPU simulation
		m_ids.erase(m_ids.begin() + newCount, m_ids.end());
		m_xPositions.erase(m_xPositions.begin() + newCount, m_xPositions.end());
		m_yPositions.erase(m_yPositions.begin() + newCount, m_yPositions.end());
		m_xVelocities.erase(m_xVelocities.begin() + newCount, m_xVelocities.end());
		m_yVelocities.erase(m_yVelocities.begin() + newCount, m_yVelocities.end());
	}

	cudaFree(m_d_xPositions);
	cudaFree(m_d_yPositions);
	cudaFree(m_d_xVelocities);
	cudaFree(m_d_yVelocities);
	cudaFree(m_d_xAccelerations);
	cudaFree(m_d_yAccelerations);

	cudaMalloc(&m_d_xPositions, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yPositions, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_xVelocities, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yVelocities, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_xAccelerations, settings->m_numBoids * sizeof(float));
	cudaMalloc(&m_d_yAccelerations, settings->m_numBoids * sizeof(float));
}

