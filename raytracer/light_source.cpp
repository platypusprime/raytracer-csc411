/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
            CSC418, SPRING 2005

        implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

/**
 * Shading code for a view ray lit by a point light. Uses Phong shading to
 * produce a colour based on intersection parameters. Assumes that
 * ray.intersection.none == false and intersection information is available.
 */
void PointLight::shade( Ray3D& ray, bool isOccluded ) {

    // Produce only ambient component if occluded
    if (isOccluded) {
        ray.col = _col_ambient * ray.intersection.mat->ambient;
        ray.col.clamp();
        return;
    }

    // compute unit vector from intersection point to light source
    Vector3D lightDir = _pos - ray.intersection.point;
    lightDir.normalize();

    // compute unit normal vector
    Vector3D normalDir = ray.intersection.normal;
    normalDir.normalize();

    // compute unit vector from intersection point to camera
    Vector3D viewDir = -1 * ray.dir;
    viewDir.normalize();

    // compute unit vector of the reflected light ray
    Vector3D reflDir = ((2 * lightDir.dot(normalDir)) * normalDir) - lightDir;

    // compute Phong intensities
    double i_diffuse = normalDir.dot(lightDir);
    i_diffuse = i_diffuse < 0 ? 0 : i_diffuse;
    double i_specular = reflDir.dot(viewDir);
    i_specular = i_specular < 0 ? 0 : i_specular;
    i_specular = pow(i_specular, ray.intersection.mat->specular_exp);

    // compute component colours based on light and material colours
    Colour c_ambient = _col_ambient * ray.intersection.mat->ambient;
    Colour c_diffuse = _col_diffuse * ray.intersection.mat->diffuse;
    Colour c_specular = _col_specular * ray.intersection.mat->specular;

    // compute final pixel color as sum of components and set ray colour
    ray.col = c_ambient + i_diffuse * c_diffuse + i_specular * c_specular;
    ray.col.clamp(); // clamp colour values in range [0.0, 1.0]
}
