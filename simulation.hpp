#pragma once

#include <memory>

class Simulation {
public:
	virtual ~Simulation() = default;

	virtual void step() = 0;
	virtual std::shared_ptr<uint32_t[]> get_data() = 0;
	virtual void resize(uint32_t width, uint32_t height) = 0;
	virtual void show_settings() = 0;

protected:
	enum DeviceType {
		CPU,
		GPU
	};

	//DeviceType m_DeviceType = CPU;
	uint32_t m_width = 0;
	uint32_t m_height = 0;

	std::shared_ptr<uint32_t[]> m_buffer = nullptr;
};