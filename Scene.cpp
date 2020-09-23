#include "Scene.h"

#include "Colour.h"
#include "ImageDisplay.h"
#include "utility.h"

Scene::Scene() : backgroundColour(0,0,0), ambientLight(0,0,0), maxRayDepth(3), renderWidth(800), renderHeight(600), filename("render.png"), camera_(), objects_(), lights_() {

}

Scene::~Scene() {

}

void Scene::render() const {
	ImageDisplay display("Render", renderWidth, renderHeight);

	const double w = double(renderWidth);
	const double h = double(renderHeight);

	for (unsigned int v = 0; v < renderHeight; ++v) {
		for (unsigned int u = 0; u < renderWidth; ++u) {
			double cu = -1 + (u + 0.5)*(2.0 / w);
			double cv = -h/w + (v + 0.5)*(2.0 / w);
			Ray ray = camera_->castRay(cu, cv);
			display.set(u, v, computeColour(ray, maxRayDepth));
		}
		display.refresh();
	}

	display.save(filename);
	display.pause(5);
}

RayIntersection Scene::intersect(const Ray& ray) const {
	RayIntersection firstHit;
	firstHit.distance = infinity;	
	for (const auto & obj : objects_) {
		for (const auto & hit : obj->intersect(ray)) {
			if (hit.distance > epsilon && hit.distance < firstHit.distance) {
				firstHit = hit;
			}
		}
	}	
	return firstHit;
}

Colour Scene::computeColour(const Ray& ray, unsigned int rayDepth) const {
	RayIntersection hitPoint = intersect(ray);
	if (hitPoint.distance == infinity) {
		return backgroundColour;
	}

	Colour hitColour(0, 0, 0);
		
	/******************************************************************
	 * Code for better lighting, shadows, and reflections goes below. *
	 ******************************************************************/

	// SHADOWS AND LIGHTING

	for (const auto & light : lights_) {
		// Calculates light's effect on the object that is hit with the ray
		if (light->getDistanceToLight(hitPoint.point) < 0) {
			// Ambient light renders flat colour without shadows
			hitColour += light->getIlluminationAt(hitPoint.point) * hitPoint.material.ambientColour;
		} 
		else {
			// Assigns the surface normal
			Vector n = hitPoint.normal;
			
			// Assigns the vector from surface hit point to light source
			Vector l = -(light->getLightDirection(hitPoint.point));
			n = n / n.norm();
			l = l / l.norm();
			
			// Assigns colours from diffuse and specular illumination
			Colour i = light->getIlluminationAt(hitPoint.point);
			Colour kd = hitPoint.material.diffuseColour;
			Colour ks = hitPoint.material.specularColour;

			// Computes surface point to view direction
			Vector v = -ray.direction;									
			v = v / v.norm();

			// Computes reflection of light source about the surface normal
			Vector r = 2 * n.dot(l) * n - l;							
			r = r / r.norm();

			// Assigns shadow rays
			Ray shadow;
			shadow.point = hitPoint.point;
			shadow.direction = l;
			RayIntersection shadowPoint = intersect(shadow);

			// Assigns the specular exponent
			double specEx = hitPoint.material.specularExponent;			
			//Dot product results, making sure its positive or 0
			double nDotL = std::max<double>(0, n.dot(l));
			double rDotV = std::max<double>(0, r.dot(v));
			
			// Gives colour and light to the object if not in shadow
			if (shadowPoint.distance > light->getDistanceToLight(hitPoint.point)) {
				hitColour += i * ((kd * nDotL) + (ks * pow(rDotV, specEx)));
			}
		}
	}

	//REFLECTIONS

	// Recursively implements reflections
	if (rayDepth > 0) {
		Ray mirrorRay;
		Vector n = hitPoint.normal;
		Vector v = -ray.direction;

		n = n / n.norm();
		v = v / v.norm();

		// Computes the angle of the ray's direction as v about n 
		mirrorRay.direction = 2 * n.dot(v) * n - v;
		mirrorRay.point = hitPoint.point;

		hitColour += hitPoint.material.mirrorColour * computeColour(mirrorRay, rayDepth - 1);
	}
	hitColour.clip();
	return hitColour;
}

bool Scene::hasCamera() const {
	return bool(camera_);
}
