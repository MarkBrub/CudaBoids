# pragma once

#include <iostream>
#include <chrono>

#include "imgui.h"
#include "imgui_internal.h"

#include "simulation.hpp"
#include "image.hpp"

class Renderer {
public:
	Renderer(Simulation& sim, ID3D11Device* device, ID3D11DeviceContext* context);

	void render();

private:
	Simulation* sim;

	ID3D11Device* m_Device = nullptr;
	ID3D11DeviceContext* m_Context = nullptr;

	Image m_Frame;
	uint32_t m_ViewportWidth = 0;
	uint32_t m_ViewportHeight = 0;
	uint64_t m_FrameCount = 0;
};