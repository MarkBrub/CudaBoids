//#include "display.hpp"
//
//
//Renderer::Renderer(const int width, const int height) {
//
//}
//
//void Renderer::frameStart() {
//
//	// Start frame timing
//	totalFrames++;
//
//	//SDL_SetRenderDrawColor(renderer, 44, 57, 75, 255);
//}
//
//void Renderer::drawBoids(std::vector<Boid>& boids) {
//	double scale = 1;
//	for (auto& boid : boids) {
//
//		Vector2 top(0, 8);
//		Vector2 left(-5, -8);
//		Vector2 right(5, -8);
//		Vector2 topP(0, 0);
//		Vector2 leftP(0, 0);
//		Vector2 rightP(0, 0);
//
//		double scaleA = scale;
//
//		//if (boid.id == 0) scaleA = 1.5;
//
//		top /= (static_cast<double>(1) / scaleA);
//		left /= (static_cast<double>(1) / scaleA);
//		right /= (static_cast<double>(1) / scaleA);
//		
//		
//		double angle = boid.velocity.angle();
//		if (boid.velocity.y >= 0) {
//			angle += std::numbers::pi;
//		}
//
//
//		topP.x = top.x * std::cos(angle) - top.y * std::sin(angle);
//		topP.y = top.y * std::cos(angle) + top.x * std::sin(angle);
//		leftP.x = left.x * std::cos(angle) - left.y * std::sin(angle);
//		leftP.y = left.y * std::cos(angle) + left.x * std::sin(angle);
//		rightP.x = right.x * std::cos(angle) - right.y * std::sin(angle);
//		rightP.y = right.y * std::cos(angle) + right.x * std::sin(angle);
//		
//
//		/*
//		// center
//		vert[0].position.x = boid.position.x + topP.x;
//		vert[0].position.y = boid.position.y - topP.y;
//
//		// left
//		vert[1].position.x = boid.position.x + leftP.x;
//		vert[1].position.y = boid.position.y - leftP.y;
//
//		// right 
//		vert[2].position.x = boid.position.x + rightP.x;
//		vert[2].position.y = boid.position.y - rightP.y;
//		
//
//		for (int x = 0; x < 3; x++) {
//			vert[x].color.r = boid.color[0];
//			vert[x].color.g = boid.color[1];
//			vert[x].color.b = boid.color[2];
//		}
//		*/
//
//		//Vector2 dir = boid.velocity / boid.velocity.length() / .05;
//
//		//SDL_RenderDrawLine(renderer, boid.position.x, boid.position.y, boid.position.x + dir.x, boid.position.y + dir.y);
//	}
//}
//
