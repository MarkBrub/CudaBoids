#include "renderer.hpp"

Renderer::Renderer(Simulation& sim, ID3D11Device* device, ID3D11DeviceContext* context) : 
    sim(&sim), m_Device(device), m_Context(context) {
    m_Frame = Image(m_Device, m_Context, ImageFormat::RGBA);
}

void Renderer::render() {
    ImGuiContext& g = *GImGui;
    ImGuiIO& io = g.IO;

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    // Create the window that the simulation can render to
	ImGui::Begin("Viewport");

    m_ViewportWidth = ImGui::GetContentRegionAvail().x;
    m_ViewportHeight = ImGui::GetContentRegionAvail().y;

    if (m_FrameCount != 0) {

        // If the viewport is uninitialized or has changed size then resize the simulation and frame
        if (m_Frame.getTextureID() == 0 || m_ViewportWidth != m_Frame.GetWidth() || m_ViewportHeight != m_Frame.GetHeight()) {
            m_Frame.resize(m_ViewportWidth, m_ViewportHeight);
            sim->resize(m_ViewportWidth, m_ViewportHeight);
        }

        // Step the simulation
        sim->step();

        // Update the frame data
        m_Frame.update_data(sim->get_data());

        // If the simulation has rendered an image then display it
        if (m_Frame.getTextureID()) {
            ImGui::Image(m_Frame.getTextureID(), {(float)m_Frame.GetWidth(), (float)m_Frame.GetHeight()});
        } else {
            ImGui::Text("No image");
            // std::cout << "No image" << std::endl;
        }

    }

    ImGui::End();

    // Create the window that the simulation can put its settings in
    ImGui::Begin("Settings");

    ImGui::Text("Average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);

    // Display the simulation specific settings
    sim->show_settings();

    ImGui::End();

    m_FrameCount++;
}
