#include "boidsCPU.hpp"
#include "boidsGPU.hpp"

BoidsCPU::BoidsCPU(BoidsData& data, size_t width, size_t height) : settings(&data) {
	m_width = width;
	m_height = height;

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);
}

void BoidsCPU::initFromGPU(std::shared_ptr<Simulation> sim) {
	// cast sim to BoidsCPU
	auto boidsGPU = std::dynamic_pointer_cast<BoidsGPU>(sim);

	if (settings->m_numBoids > 2000) {
		settings->m_numBoids = 2000;
	}

	m_boids.resize(settings->m_numBoids);

	for (size_t x = 0; x < settings->m_numBoids; x++) {
		m_boids[x].id = boidsGPU->m_ids[x];
		m_boids[x].position.x = boidsGPU->m_xPositions[x];
		m_boids[x].position.y = boidsGPU->m_yPositions[x];
		m_boids[x].velocity.x = boidsGPU->m_xVelocities[x];
		m_boids[x].velocity.y = boidsGPU->m_yVelocities[x];

		m_boids[x].color = Util::toUint32(settings->m_Colors[x % 4]);
	}
}

void BoidsCPU::step() {
	if (m_boids.size() != settings->m_numBoids) {
		changePopulationSize(settings->m_numBoids);
	}

	m_visionRadiusSquared = settings->m_visionRadius * settings->m_visionRadius;
	m_avoidanceRadiusSquared = settings->m_avoidanceRadius * settings->m_avoidanceRadius;

	std::vector<Vector2> accelerations(settings->m_numBoids);

	for (auto& boid : m_boids) {
		accelerations[boid.id] = acceleration(boid);
	}

	for (auto& boid : m_boids) {
		boid.velocity += accelerations[boid.id];

		if (boid.velocity.length() > settings->m_speedCap) {
			boid.velocity = (boid.velocity / boid.velocity.length()) * settings->m_speedCap;
		}

		boid.position += boid.velocity;
		bound(boid);
	}
}

std::shared_ptr<uint32_t[]> BoidsCPU::get_data() {
	std::fill(m_buffer.get(), m_buffer.get() + m_width * m_height, settings->m_BackgroundColor);

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
				m_buffer[position] = boid.color;
			}
		}
	}

	return m_buffer;
}

void BoidsCPU::resize(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;

	m_buffer = std::shared_ptr<uint32_t[]>(new uint32_t[m_width * m_height]);
}

void BoidsCPU::show_settings() {
	if (settings->changed) {
		// make sure the number of boids doesnt get too high
		// CPU cant handel that many
		if (settings->m_numBoids > 2000) settings->m_numBoids = 2000;
	}
}

void BoidsCPU::changePopulationSize(const int newCount) {
	if (newCount > m_boids.size()) {
		for (size_t x = m_boids.size(); x < newCount; x++) {
			m_boids.emplace_back(m_height, m_width);
			m_boids[x].id = (int)x;
			m_boids[x].color = Util::toUint32(settings->m_Colors[x % 4]);
		}
	} else {
		m_boids.erase(m_boids.begin() + newCount, m_boids.end());
	}
}


Vector2 BoidsCPU::acceleration(Boid& self) {
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
		if (distanceSquared < m_visionRadiusSquared) {
			//if ((boid.position - self.position).normalize() * self.velocity.normalize() < m_fieldOfViewRange) continue;
			if (distanceSquared < m_avoidanceRadiusSquared) {
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

	seperationVec *= settings->m_seperation;

	if (closeBoids > 0) {
		alignmentVec /= closeBoids;
		alignmentVec = (alignmentVec - (self.velocity / closeBoids)) * settings->m_alignment;

		cohesionVec /= closeBoids;
		cohesionVec = (cohesionVec - self.position) * settings->m_cohesion;
	}

	return seperationVec + alignmentVec + cohesionVec;
}

void BoidsCPU::bound(Boid& boid) {
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

	float correction = (float)settings->m_speedCap * .05f;

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