#include "image.hpp"

Image::Image(ID3D11Device* device, ID3D11DeviceContext* context, ImageFormat format) :
	m_devicePtr(device), m_contextPtr(context), m_format(format) {
	m_texDesc.MipLevels = 1;
	m_texDesc.ArraySize = 1;

	// Determine DXGI format based on ImageFormat
	switch (m_format) {
	case ImageFormat::RGBA:
		m_texDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		break;
	case ImageFormat::RGBA32F:
		m_texDesc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
		break;
	default:
		// Handle other formats if needed
		break;
	}

	m_texDesc.SampleDesc.Count = 1;
	m_texDesc.Usage = D3D11_USAGE_DEFAULT;
	m_texDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

	m_srvDesc.Format = m_texDesc.Format;
	m_srvDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	m_srvDesc.Texture2D.MipLevels = 1;
}

// Destructor implementation
Image::~Image() {
    // Release resources
	if (m_TextureView) {
		m_TextureView->Release();
	}
        
	if (m_Texture) {
		m_Texture->Release();
	}
}

void Image::update_data(std::shared_ptr<uint32_t[]> data) {
	m_contextPtr->UpdateSubresource(m_Texture, 0, nullptr, data.get(), m_initData.SysMemPitch, 0);
}

void Image::resize(uint32_t width, uint32_t height) {
    if (width == m_width && height == m_height) {
		return;
	}

	m_width = width;
	m_height = height;

    if (m_Texture) {
		m_Texture->Release();
		m_Texture = nullptr;
	}

    if (m_TextureView) {
		m_TextureView->Release();
		m_TextureView = nullptr;
	}

	// Set up initial data
	m_initData.SysMemPitch = m_width * (m_format == ImageFormat::RGBA ? 4 : sizeof(float));

	// Set up texture description
	m_texDesc.Width = width;
	m_texDesc.Height = height;

	HRESULT hr;

	hr = m_devicePtr->CreateTexture2D(&m_texDesc, nullptr, &m_Texture);
	if (!m_Texture || FAILED(hr)) {
		std::cerr << "Failed to create texture" << std::endl;
		return;
	}

	hr = m_devicePtr->CreateShaderResourceView(m_Texture, &m_srvDesc, &m_TextureView);
	if (!m_TextureView || FAILED(hr)) {
		std::cerr << "Failed to create shader resource view" << std::endl;
		return;
	}
}
