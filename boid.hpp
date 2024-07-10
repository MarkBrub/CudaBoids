#pragma once

#include "util.hpp"
#include "vector2.hpp"

struct Boid {
	int id = 0;
	Vector2 position;
	Vector2 velocity;
	uint32_t color = 0;

	// default boid shape
	// Order: Top, Bottom left, Bottom right
	const static std::vector<Vector2> BasePoints;

	Boid() = default;

	Boid(const int maxHeight, const int maxWidth) {
		position.x = Util::getRandomNumber(maxWidth * .1f, maxWidth * .9f);
		position.y = Util::getRandomNumber(maxHeight * .1f, maxHeight * .9f);
		velocity.x = Util::getRandomNumber(-3, 3);
		velocity.y = Util::getRandomNumber(-3, 3);
	}

	Boid& operator=(const Boid& rhs) {
		position = rhs.position;
		velocity = rhs.velocity;
		color = rhs.color;

		return *this;
	}

	void GetBoidShape(Vector2& top, Vector2& left, Vector2& right, float& scale) const {
		static const float pi = 3.14159265358979323846f;
		float angle = velocity.angle();
		if (velocity.y >= 0) {
			// angle += (float)std::numbers::pi;
			angle += pi;
		}

		top.x = BasePoints[0].x * std::cos(angle) - BasePoints[0].y * std::sin(angle);
		top.y = BasePoints[0].y * std::cos(angle) + BasePoints[0].x * std::sin(angle);
		left.x = BasePoints[1].x * std::cos(angle) - BasePoints[1].y * std::sin(angle);
		left.y = BasePoints[1].y * std::cos(angle) + BasePoints[1].x * std::sin(angle);
		right.x = BasePoints[2].x * std::cos(angle) - BasePoints[2].y * std::sin(angle);
		right.y = BasePoints[2].y * std::cos(angle) + BasePoints[2].x * std::sin(angle);

		top *= scale;
		left *= scale;
		right *= scale;

		top.x *= -1;
		left.x *= -1;
		right.x *= -1;

		top += position;
		left += position;
		right += position;
	}

	// auto operator<=>(const Boid&) const = default;

	auto operator==(const Boid& rhs) const {
		return id == rhs.id;
	}

	auto operator!=(const Boid& rhs) const {
		return !(*this == rhs);
	}
};