#pragma once

#include <random>
// #include <numbers>


//https://stackoverflow.com/questions/32071721/error-in-using-mt19937-with-random-device-as-static-class-members
struct Util {
	static std::random_device   m_rd;
	static std::mt19937         m_rng;
public:
	static float getRandomNumber(const float& rangeStart, const float& rangeEnd);
	static uint32_t toUint32(const std::vector<uint32_t>& rgb);
	
};