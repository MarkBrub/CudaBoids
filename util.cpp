#include "Util.hpp"

std::random_device   Util::m_rd;
std::mt19937         Util::m_rng(Util::m_rd());

//between start and end
//incluseive of start but not end
float Util::getRandomNumber(const float& rangeStart, const float& rangeEnd) {
	std::uniform_real_distribution<> randomizer(rangeStart, rangeEnd);
	return randomizer(m_rng);
}

uint32_t Util::toUint32(const std::vector<uint32_t>& rgb) {
	uint32_t out = 0xFF000000;

	out |= rgb[2] << 16;
	out |= rgb[1] << 8;
	out |= rgb[0];

	return out;
}

