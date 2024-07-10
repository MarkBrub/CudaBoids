#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include "math.h"

#include "boid.hpp"

__constant__ struct BoidSettings {
	int width;
	int height;
	float seperation;
	float alignment;
	float cohesion;
	int speedCap;
	int visionRadius;
	int avoidanceRadius;
	int fieldOfView;
	float scale;
} settings;

__constant__ struct BasePoints {
	float x;
	float y;
} basePoints[3];

__device__ float calcDistSquared(float x1, float x2, float y1, float y2) {
	return ((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2));
}

void copySettingsToGPU(int width, int height, float seperation, float alignment, float cohesion, int speedCap, int visionRadius, int avoidanceRadius, float scale) {
	BoidSettings tempSettings;
	tempSettings.width = width;
	tempSettings.height = height;
	tempSettings.seperation = seperation;
	tempSettings.alignment = alignment;
	tempSettings.cohesion = cohesion;
	tempSettings.speedCap = speedCap;
	tempSettings.visionRadius = visionRadius;
	tempSettings.avoidanceRadius = avoidanceRadius;
	tempSettings.scale = scale;

	BasePoints tempBasePoints[3];
	tempBasePoints[0].x = Boid::BasePoints[0].x;
	tempBasePoints[0].y = Boid::BasePoints[0].y;
	tempBasePoints[1].x = Boid::BasePoints[1].x;
	tempBasePoints[1].y = Boid::BasePoints[1].y;
	tempBasePoints[2].x = Boid::BasePoints[2].x;
	tempBasePoints[2].y = Boid::BasePoints[2].y;

	cudaMemcpyToSymbol(settings, &tempSettings, sizeof(BoidSettings));
	cudaMemcpyToSymbol(basePoints, &tempBasePoints, sizeof(BasePoints) * 3);
}

__global__ void calculateAccelerationKernel(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids) {
	int index = blockIdx.x * blockDim.x + threadIdx.x;

	float seperationX = 0;
	float seperationY = 0;
	float alignmentX = 0;
	float alignmentY = 0;
	float cohesionX = 0;
	float cohesionY = 0;

	int closeBoids = 0;

	int avoidanceRadiusSquared = settings.avoidanceRadius * settings.avoidanceRadius;
	int visionRadiusSquared = settings.visionRadius * settings.visionRadius;

	float xPos = xPositions[index];
	float yPos = yPositions[index];

	for (int x = 0; x < numBoids; x++) {
		if (x == index) continue;

		float distSquared = calcDistSquared(xPos, xPositions[x], yPos, yPositions[x]);
		
		if (distSquared < visionRadiusSquared) {
			if (distSquared < avoidanceRadiusSquared) {
				seperationX += xPos - xPositions[x];
				seperationY += yPos - yPositions[x];
			}

			// if (index % 4 != x % 4) continue;
			if ((index & 3) != (x & 3)) continue;

			alignmentX += xVelocities[x];
			alignmentY += yVelocities[x];
			cohesionX += xPositions[x];
			cohesionY += yPositions[x];

			closeBoids++;
		}
	}

	seperationX *= settings.seperation;
	seperationY *= settings.seperation;

	if (closeBoids > 0) {
		alignmentX /= closeBoids;
		alignmentY /= closeBoids;

		alignmentX = (alignmentX - (xVelocities[index]) / closeBoids) * settings.alignment;
		alignmentY = (alignmentY - (yVelocities[index]) / closeBoids) * settings.alignment;
		
		cohesionX /= closeBoids;
		cohesionY /= closeBoids;

		cohesionX = (cohesionX - xPos) * settings.cohesion;
		cohesionY = (cohesionY - yPos) * settings.cohesion;
	}

	xAccelerations[index] = seperationX + alignmentX + cohesionX;
	yAccelerations[index] = seperationY + alignmentY + cohesionY;
}

__global__ void updatePositionKernel(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids) {
	int index = blockIdx.x * blockDim.x + threadIdx.x;

	// Update velocity
	xVelocities[index] += xAccelerations[index];
	yVelocities[index] += yAccelerations[index];

	// Cap velocity
	float velocity = sqrtf((xVelocities[index] * xVelocities[index]) + (yVelocities[index] * yVelocities[index]));
	if (velocity > settings.speedCap) {
		xVelocities[index] = (xVelocities[index] / velocity) * settings.speedCap;
		yVelocities[index] = (yVelocities[index] / velocity) * settings.speedCap;
	}

	// Update position
	xPositions[index] += xVelocities[index];
	yPositions[index] += yVelocities[index];

	// Bound position
	/*
	if (xPositions[index] < 0) {
		xPositions[index] = settings.width;
	} else if (xPositions[index] > settings.width) {
		xPositions[index] = 0;
	}

	if (yPositions[index] < 0) {
		yPositions[index] = settings.height;
	} else if (yPositions[index] > settings.height) {
		yPositions[index] = 0;
	}
	*/

	float correction = (float)settings.speedCap * .1f;

	if (xPositions[index] < settings.width * .05) {
		xVelocities[index] += correction;
	} else if (xPositions[index] > settings.width * .95) {
		xVelocities[index] -= correction;
	}

	if (yPositions[index] < settings.height * .05) {
		yVelocities[index] += correction;
	} else if (yPositions[index] > settings.height * .95) {
		yVelocities[index] -= correction;
	}
}

__global__ void renderBoidsKernel(uint32_t* data, float* xPositions, float* yPositions, float* xVelocities, float* yVelocities) {
	int index = blockIdx.x * blockDim.x + threadIdx.x;

	float angle = yVelocities[index] ? atan2f(xVelocities[index], yVelocities[index]) : 0;
}

void calculateAcceleration(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids) {
	// Calculate the number of blocks needed
	int blockSize = 256;
	int numBlocks = (numBoids + blockSize - 1) / blockSize;

	dim3 blockDim(blockSize);
	dim3 gridDim(numBlocks);
	
	// Launch the kernel
	calculateAccelerationKernel<<<gridDim, blockDim>>>(xPositions, yPositions, xVelocities, yVelocities, xAccelerations, yAccelerations, numBoids);

	// Wait for all threads to finish
	cudaDeviceSynchronize();
}

void updatePosition(float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, float* xAccelerations, float* yAccelerations, int numBoids) {
	// Calculate the number of blocks needed
	int blockSize = 256;
	int numBlocks = (numBoids + blockSize - 1) / blockSize;

	dim3 blockDim(blockSize);
	dim3 gridDim(numBlocks);

	// Launch the kernel
	updatePositionKernel<<<gridDim, blockDim>>>(xPositions, yPositions, xVelocities, yVelocities, xAccelerations, yAccelerations, numBoids);

	// Wait for all threads to finish
	cudaDeviceSynchronize();
}

void renderBoids(uint32_t* data, float* xPositions, float* yPositions, float* xVelocities, float* yVelocities, int numBoids) {
	// Calculate the number of blocks needed
	int blockSize = 256;
	int numBlocks = (numBoids + blockSize - 1) / blockSize;

	dim3 blockDim(blockSize);
	dim3 gridDim(numBlocks);

	// Launch the kernel
	renderBoidsKernel<<<gridDim, blockDim>>>(data, xPositions, yPositions, xVelocities, yVelocities);

	// Wait for all threads to finish
	cudaDeviceSynchronize();
}