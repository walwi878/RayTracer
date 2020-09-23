#include "Plane.h"

#include "utility.h"

Plane::Plane() : Object() {

}

Plane::Plane(const Plane& plane) : Object(plane) {

}

Plane::~Plane() {

}

Plane& Plane::operator=(const Plane& plane) {
	if (this != &plane) {
		Object::operator=(plane);
	}
	return *this;
}

std::vector<RayIntersection> Plane::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	/**********************************************
	 * Code for Ray-Plane intersection goes here. *
     **********************************************/

    Ray inverseRay = transform.applyInverse(ray);

    // Name intersects
    double z0 = inverseRay.point(2);
    double dz = inverseRay.direction(2);
    double t = -z0 / dz;

    RayIntersection hit;

    // Check if dividing by zero
    if (std::abs(dz) < epsilon) {
        return result;
    }

    if (t > 0) {
        hit.point = inverseRay.point + t * inverseRay.direction;

        // Check x and y are in range [-1,1], then calculate hit point's normal, material, and distance
        if ((hit.point(0) > -1 && hit.point(0) < 1) && (hit.point(1) > -1 && hit.point(1) < 1)) {
            
            hit.normal = Normal(0, 0, 1);
            hit.normal = transform.apply(Normal(hit.normal));

            hit.point = transform.apply(Point(hit.point));

            hit.material = material;

            hit.distance = (hit.point - ray.point).norm();
            result.push_back(hit);
        }
    }
    return result;
}