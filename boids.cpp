#include "boids.hpp"
#include "boidsCPU.hpp"
#include "boidsGPU.hpp"

Boids::Boids() {
	m_boids = std::make_shared<BoidsCPU>(m_data, m_width, m_height);
}

void Boids::step() {
	std::iostream::sync_with_stdio(false);
	auto start = std::chrono::high_resolution_clock::now();
	m_boids->step();
	auto end = std::chrono::high_resolution_clock::now();
	m_stepTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

std::shared_ptr<uint32_t[]> Boids::get_data() {
	return m_boids->get_data();
}

void Boids::resize(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;

	// The m_boids buffer is used instead
	// m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);

	m_boids->resize(width, height);
}

void Boids::show_settings() {
	bool deviceChanged = false;

	ImGui::Text("Step Time: %d ms", static_cast<int>(std::floor(m_stepTime)));


	m_data.changed = false;

	m_data.changed |= ImGui::SliderInt(	 "Boid Count",		 &m_data.m_numBoids, 0, m_data.m_maxBoids, "%d", 0);
	m_data.changed |= ImGui::SliderFloat("Seperation",		 &m_data.m_seperation, 0, m_data.m_maxSeperation, "%.4f", 0);
	m_data.changed |= ImGui::SliderFloat("Alignment",		 &m_data.m_alignment, 0, m_data.m_maxAlignment, "%.4f", 0);
	m_data.changed |= ImGui::SliderFloat("Cohesion",		 &m_data.m_cohesion, 0, m_data.m_maxCohesion, "%.4f", 0);
	m_data.changed |= ImGui::SliderInt(	 "Max Speed",		 &m_data.m_speedCap, 0, m_data.m_maxSpeedCap, "%d", 0);
	m_data.changed |= ImGui::SliderInt(	 "Vision Radius",	 &m_data.m_visionRadius, 20, m_data.m_maxVisionRadius, "%d", 0);
	m_data.changed |= ImGui::SliderInt(  "Avoidance Radius", &m_data.m_avoidanceRadius, 2, m_data.m_maxAvoidanceRadius, "%d", 0);
	m_data.changed |= ImGui::SliderFloat("Scale",			 &m_data.m_scale, 0, m_data.m_maxScale, "%.2f", 0);

	deviceChanged |= ImGui::Checkbox("Use GPU", &m_useGPU);

	// get cuda device name
	cudaDeviceProp prop{};
	cudaGetDeviceProperties(&prop, 0);
	ImGui::Text("GPU: %s", prop.name);

	// get cuda device memory
	size_t freeMem, totalMem = 0;
	cudaMemGetInfo(&freeMem, &totalMem);
	ImGui::Text("Free Memory: %d MB", freeMem / 1024 / 1024);
	ImGui::Text("Total Memory: %d MB", totalMem / 1024 / 1024);

	// Swap device if needed
	if (deviceChanged) {
		if (m_useGPU) {
			// create GPU simulation
			auto boidsGPU = std::make_shared<BoidsGPU>(m_data, m_width, m_height);

			// transfer state from CPU to GPU
			boidsGPU->initFromCPU(m_boids);

			// set GPU as simulation
			m_boids = boidsGPU;
		} else {
			// create CPU simulation
			auto boidsCPU = std::make_shared<BoidsCPU>(m_data, m_width, m_height);

			// transfer state from GPU to CPU
			boidsCPU->initFromGPU(m_boids);

			// set CPU as simulation
			m_boids = boidsCPU;
		}
	}

	m_boids->show_settings();
}

/*
Boids::Boids() {}

Boids::Boids(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);
}

Boids::~Boids() {
	if (m_DeviceType == GPU) {
		cudaFree(m_d_data);
		cudaFree(m_d_xPositions);
		cudaFree(m_d_yPositions);
		cudaFree(m_d_xVelocities);
		cudaFree(m_d_yVelocities);
		cudaFree(m_d_xAccelerations);
		cudaFree(m_d_yAccelerations);
	}
}

void Boids::step() {
	if (m_useGPU != m_DeviceType) {
		swapDevice();
	}

	if (m_boids.size() != m_numBoids) {
		changePopulationSize(m_numBoids);
	}

	if (m_DeviceType == CPU) {
		stepCPU();
	} else {
		stepGPU();
	}
}

std::shared_ptr<uint32_t[]> Boids::get_data() {
	if (m_DeviceType == CPU) {
		renderCPU();
	} else {
		renderGPU();
	}

	return m_buffer;
}

void Boids::resize(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);

	if (m_useGPU) {
		cudaFree(m_d_data);
		cudaMalloc(&m_d_data, m_width * m_height * sizeof(uint32_t));
	}

	copySettingsToGPU(m_width, m_height,
		m_seperation, m_alignment, m_cohesion, m_speedCap,
		m_visionRadius, m_avoidanceRadius, m_scale);
}

void Boids::show_settings() {
	bool changed = false;
	
	changed |= ImGui::SliderInt("Boid Count", &m_numBoids, 0, m_maxBoids, "%d", 0);
	changed |= ImGui::SliderFloat("Seperation", &m_seperation, 0, m_maxSeperation, "%.4f", 0);
	changed |= ImGui::SliderFloat("Alignment", &m_alignment, 0, m_maxAlignment, "%.4f", 0);
	changed |= ImGui::SliderFloat("Cohesion", &m_cohesion, 0, m_maxCohesion, "%.4f", 0);
	changed |= ImGui::SliderInt("Max Speed", &m_speedCap, 0, m_maxSpeedCap, "%d", 0);
	changed |= ImGui::SliderInt("Vision Radius", &m_visionRadius, 20, m_maxVisionRadius, "%d", 0);
	changed |= ImGui::SliderInt("Avoidance Radius", &m_avoidanceRadius, 5, m_maxAvoidanceRadius, "%d", 0);
	changed |= ImGui::SliderFloat("Scale", &m_scale, 0, m_maxScale, "%.2f", 0);
	
	ImGui::Checkbox("Use GPU", &m_useGPU);

	// get cuda device name
	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, 0);
	ImGui::Text("GPU: %s", prop.name);

	// get cuda device memory
	size_t freeMem, totalMem;
	cudaMemGetInfo(&freeMem, &totalMem);
	ImGui::Text("Free Memory: %d MB", freeMem / 1024 / 1024);
	ImGui::Text("Total Memory: %d MB", totalMem / 1024 / 1024);

	if (changed) {
		if (m_useGPU) {
			copySettingsToGPU(m_width, m_height,
				m_seperation, m_alignment, m_cohesion, m_speedCap,
				m_visionRadius, m_avoidanceRadius, m_scale);
		}

		// Update cached values
		m_VisionRadiusSquared = m_visionRadius * m_visionRadius;
		m_AvoidanceRadiusSquared = m_avoidanceRadius * m_avoidanceRadius;
	}
}

void Boids::swapDevice() {
	if (m_useGPU) {
		m_DeviceType = GPU;

		for (const auto& boid : m_boids) {
			m_xPositions[boid.id] = boid.position.x;
			m_yPositions[boid.id] = boid.position.y;
			m_xVelocities[boid.id] = boid.velocity.x;
			m_yVelocities[boid.id] = boid.velocity.y;
		}

		cudaMalloc(&m_d_xPositions, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yPositions, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_xVelocities, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yVelocities, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_xAccelerations, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yAccelerations, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_data, m_width * m_height * sizeof(uint32_t));

		copySettingsToGPU(m_width, m_height,
			m_seperation, m_alignment, m_cohesion, m_speedCap,
			m_visionRadius, m_avoidanceRadius, m_scale);

	} else {
		m_DeviceType = CPU;

		for (auto& boid : m_boids) {
			boid.position.x = m_xPositions[boid.id];
			boid.position.y = m_yPositions[boid.id];
			boid.velocity.x = m_xVelocities[boid.id];
			boid.velocity.y = m_yVelocities[boid.id];
		}

		cudaFree(m_d_data);
		cudaFree(m_d_xPositions);
		cudaFree(m_d_yPositions);
		cudaFree(m_d_xVelocities);
		cudaFree(m_d_yVelocities);
		cudaFree(m_d_xAccelerations);
		cudaFree(m_d_yAccelerations);
	}
}

void Boids::changePopulationSize(const int newCount) {
	if (newCount > m_boids.size()) {
		for (size_t x = m_boids.size(); x < newCount; x++) {
			// Add boids to CPU simulation
			m_boids.emplace_back(m_height, m_width);
			m_boids[x].id = (int)x;
			m_boids[x].color = Util::toUint32(m_Colors[x % 4]);

			// Add boids to GPU simulation
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

	if (m_useGPU) {
		cudaFree(m_d_xPositions);
		cudaFree(m_d_yPositions);
		cudaFree(m_d_xVelocities);
		cudaFree(m_d_yVelocities);
		cudaFree(m_d_xAccelerations);
		cudaFree(m_d_yAccelerations);

		cudaMalloc(&m_d_xPositions, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yPositions, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_xVelocities, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yVelocities, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_xAccelerations, m_numBoids * sizeof(float));
		cudaMalloc(&m_d_yAccelerations, m_numBoids * sizeof(float));
	}
}

void Boids::stepCPU() {
	std::vector<Vector2> accelerations(m_numBoids);

	for (auto& boid : m_boids) {
		accelerations[boid.id] = accelerationCPU(boid);
	}

	for (auto& boid : m_boids) {
		boid.velocity += accelerations[boid.id];

		if (boid.velocity.length() > m_speedCap) {
			boid.velocity = (boid.velocity / boid.velocity.length()) * m_speedCap;
		}

		boid.position += boid.velocity;
		boundCPU(boid);
	}
}

void Boids::renderCPU() {
	std::fill(m_buffer.get(), m_buffer.get() + m_width * m_height, m_BackgroundColor);

	for (const auto& boid : m_boids) {
		Vector2 top;
		Vector2 left;
		Vector2 right;

		boid.GetBoidShape(top, left, right, m_scale);

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
				m_buffer[position] = boid.color;
			}
		}
	}
}

Vector2 Boids::accelerationCPU(Boid& self) {
	Vector2 seperationVec;
	Vector2 alignmentVec;
	Vector2 cohesionVec;

	float closeBoids = 0;
	//float reallyCloseBoids = 0;

	for (const auto& boid : m_boids) {
		if (boid.id == self.id) continue;
		//if (abs(boid.position.x - self.position.x) > m_visionRadius) continue;
		//if (abs(boid.position.y - self.position.y) > m_visionRadius) continue;

		float distanceSquared = distSquared(boid.position, self.position);

		// For all other boids in perception range
		if (distanceSquared < m_VisionRadiusSquared) {
			//if ((boid.position - self.position).normalize() * self.velocity.normalize() < m_fieldOfViewRange) continue;
			if (distanceSquared < m_AvoidanceRadiusSquared) {
				seperationVec += self.position - boid.position;
				//reallyCloseBoids++;
			}

			if (self.id % 4 != boid.id % 4) continue;
			// if (self.id == 0) continue;

			alignmentVec += boid.velocity;
			cohesionVec += boid.position;
			closeBoids++;
		}
	}

	seperationVec *= m_seperation;

	if (closeBoids > 0) {
		alignmentVec /= closeBoids;
		alignmentVec = (alignmentVec - (self.velocity / closeBoids)) * m_alignment;

		cohesionVec /= closeBoids;
		cohesionVec = (cohesionVec - self.position) * m_cohesion;
	}

	return seperationVec + alignmentVec + cohesionVec;
}

void Boids::boundCPU(Boid& boid) {
	//if (boid.position.x < 0) {
	//	boid.position.x = m_width;
	//} else if (boid.position.x > m_width) {
	//	boid.position.x = 0;
	//}

	//if (boid.position.y < 0) {
	//	boid.position.y = m_height;
	//} else if (boid.position.y > m_height) {
	//	boid.position.y = 0;
	//}

	float correction = (float)m_speedCap * .1f;

	if (boid.position.x < m_height * .05) {
		boid.velocity.x += 1;
	} else if (boid.position.x > m_width * .95) {
		boid.velocity.x -= 1;
	}

	if (boid.position.y < m_height * .05) {
		boid.velocity.y += 1;
	} else if (boid.position.y > m_height * .95) {
		boid.velocity.y -= 1;
	}
}

void Boids::stepGPU() {
	cudaMemcpy(m_d_xPositions, m_xPositions.data(), m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_yPositions, m_yPositions.data(), m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_xVelocities, m_xVelocities.data(), m_numBoids * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_d_yVelocities, m_yVelocities.data(), m_numBoids * sizeof(float), cudaMemcpyHostToDevice);

	calculateAcceleration(m_d_xPositions, m_d_yPositions, m_d_xVelocities, m_d_yVelocities, m_d_xAccelerations, m_d_yAccelerations, m_numBoids);
	updatePosition(m_d_xPositions, m_d_yPositions, m_d_xVelocities, m_d_yVelocities, m_d_xAccelerations, m_d_yAccelerations, m_numBoids);
}

void Boids::renderGPU() {
	//copy data back to CPU
	cudaMemcpy(m_xPositions.data(), m_d_xPositions, m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_yPositions.data(), m_d_yPositions, m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_xVelocities.data(), m_d_xVelocities, m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_yVelocities.data(), m_d_yVelocities, m_numBoids * sizeof(float), cudaMemcpyDeviceToHost);

	// stepCPU();

	// move data back to boids
	for (size_t x = 0; x < m_numBoids; x++) {
		m_boids[x].position.x = m_xPositions[x];
		m_boids[x].position.y = m_yPositions[x];
		m_boids[x].velocity.x = m_xVelocities[x];
		m_boids[x].velocity.y = m_yVelocities[x];
	}

	renderCPU();
}
*/