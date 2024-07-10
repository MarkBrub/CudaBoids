#pragma once

#include <memory>
#include <iostream>

#include <d3d11.h>

#include "util.hpp"

enum class ImageFormat {
	None = 0,
	RGBA,
	RGBA32F
};

// A DirectX 11 texture
class Image {
public:

	Image() = default;
	Image(ID3D11Device* device, ID3D11DeviceContext* context, ImageFormat format);
	~Image();

	// Get a value to pass to imgui's image function
	void* getTextureID() const { return (void*)m_TextureView; }

	uint32_t GetWidth() const { return m_width; }
	uint32_t GetHeight() const { return m_height; }

	void update_data(std::shared_ptr<uint32_t[]> data);
	void resize(uint32_t width, uint32_t height);

private:

	uint32_t m_width = 0;
	uint32_t m_height = 0;
	ImageFormat m_format = ImageFormat::None;

	ID3D11Device* m_devicePtr = nullptr;
	ID3D11DeviceContext* m_contextPtr = nullptr;

	D3D11_SUBRESOURCE_DATA m_initData = {};
	D3D11_TEXTURE2D_DESC m_texDesc = {};
	D3D11_SHADER_RESOURCE_VIEW_DESC m_srvDesc = {};
	ID3D11Texture2D* m_Texture = nullptr;
	ID3D11ShaderResourceView* m_TextureView = nullptr;
};