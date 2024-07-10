#pragma once

#include <cmath>

struct Vector2 {
	float x = 0;
	float y = 0;

	Vector2() = default;
	Vector2(const Vector2& v) = default;
	Vector2(float a, float b) : x(a), y(b) {};

	Vector2& operator=(const Vector2& rhs) = default;

	float length() const {
		return sqrt((x * x) + (y * y));
	}

	float angle() const {
		if (y == 0) return 0;
		return std::atan(x / y);
	}

	Vector2 normalize() const {
		return *this / length();
	}

	Vector2& operator+=(const Vector2& rhs) {
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	Vector2& operator-=(const Vector2& rhs) {
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vector2& operator*=(const float& rhs) {
		x *= rhs;
		y *= rhs;
		return *this;
	}

	Vector2& operator/=(const float& rhs) {
		x /= rhs;
		y /= rhs;
		return *this;
	}

	// Dot product
	float operator*(const Vector2& rhs) const {
		return (x * rhs.x) + (y * rhs.y);
	}

	// auto operator<=>(const Vector2&) const = default;

	friend Vector2 operator+(Vector2 lhs, const Vector2& rhs) {
		return lhs += rhs;
	}

	friend Vector2 operator-(Vector2 lhs, const Vector2& rhs) {
		return lhs -= rhs;
	}

	friend Vector2 operator/(Vector2 lhs, const float& rhs) {
		return lhs /= rhs;
	}

	friend Vector2 operator*(Vector2 lhs, const float& rhs) {
		return lhs *= rhs;
	}

	friend float dist(const Vector2& lhs, const Vector2& rhs) {
		return sqrt(((lhs.x - rhs.x) * (lhs.x - rhs.x)) + ((lhs.y - rhs.y) * (lhs.y - rhs.y)));
	}

	friend float distSquared(const Vector2& lhs, const Vector2& rhs) {
		return ((lhs.x - rhs.x) * (lhs.x - rhs.x)) + ((lhs.y - rhs.y) * (lhs.y - rhs.y));
	}

	friend float sign(const Vector2& v1, const Vector2& v2, const Vector2& v3) {
		return (v1.x - v3.x) * (v2.y - v3.y) - (v2.x - v3.x) * (v1.y - v3.y);
	}

	friend bool inTriangle(const Vector2& pt, const Vector2& v1, const Vector2& v2, const Vector2& v3) {
		float d1, d2, d3;
		bool has_neg, has_pos;

		d1 = sign(pt, v1, v2);
		d2 = sign(pt, v2, v3);
		d3 = sign(pt, v3, v1);

		has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
		has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

		return !(has_neg && has_pos);
	}
};