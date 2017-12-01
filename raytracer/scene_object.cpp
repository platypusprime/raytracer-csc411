/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
            CSC418, SPRING 2005

        implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

/**
* Intersection code for UnitSquare, which is defined on the xy-plane, with
* vertices (0.5, 0.5, 0), (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and
* normal (0, 0, 1).
*
* @param ray The ray to check intersection for
* @param worldToModel The transformation matrix taking points from world space to sphere space
* @param modelToWorld The transformation matrix taking points from sphere space to world space
* @return true if the ray's intersection was overwritten, false otherwise
*/
bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
        const Matrix4x4& modelToWorld ) {

    // transform ray into object space
    Point3D origin_model = worldToModel * ray.origin;
    Vector3D dir_model = worldToModel * ray.dir;

    // handle case where the ray is parallel with the xy-plane
    // no need for epsilon here; small z values handled in calculation of intersection
    if (dir_model[2] == 0.0) {
        return false;
    }

    // compute the ray's (singular) intersection with the xy-plane
    double t_model = -origin_model[2] / dir_model[2];
    Point3D p_intersect = origin_model + t_model * dir_model;

    // check if the point of intersection lies within the unit square
    if (abs(p_intersect[0]) > 0.5 || abs(p_intersect[1]) > 0.5) {
        return false;
    }

    // apply world transformation to intersection point and t-value
    p_intersect = modelToWorld * p_intersect;
    double t_world = (p_intersect - ray.origin)[0] / ray.dir[0];

    // check against prior intersection
    if (!ray.intersection.none && t_world >= ray.intersection.t_value) {
        return false;
    }

    // fill in intersection struct
    ray.intersection.none = false;
    ray.intersection.t_value = t_world;
    ray.intersection.point = p_intersect;
    ray.intersection.normal = transNorm(worldToModel, Vector3D(0, 0, 1));
    return true;
}

/**
 * Intersection code for UnitSphere, which is centered about the origin.
 *
 * @param ray The ray to check intersection for
 * @param worldToModel The transformation matrix taking points from world space to sphere space
 * @param modelToWorld The transformation matrix taking points from sphere space to world space
 * @return true if the ray's intersection was overwritten, false otherwise
 */
bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
        const Matrix4x4& modelToWorld ) {

    // transform ray into object space
    Point3D origin_model = worldToModel * ray.origin;
    Vector3D dir_model = worldToModel * ray.dir;

    // convert origin to vector for dot products
    Vector3D origin_model_vec = Vector3D(origin_model);

    // normalize ray direction vector to simplify calculation
    dir_model.normalize();

    // calculate intersection (solve quadratic equation for t)
    // (o.x + t * d.x) ^ 2 + (o.y + t * d.y) ^ 2 + (o.z + t * d.z) ^ 2 = 1
    double d_dot_o = dir_model.dot(origin_model_vec);
    double t_plus_minus = pow(d_dot_o, 2) - (origin_model_vec.lengthsq() - 1);

    // check for intersection
    if (t_plus_minus < 0) {
        return false;
    } else {
        // take square root of non-negative +/- term (won't be NaN)
        t_plus_minus = sqrt(t_plus_minus);

        double t_model; // the t-value for intersections in model space

        // select intersection (note that t_pos is always larger than t_neg)
        double t_neg = 0.0, t_pos = 0.0;
        if ((t_neg = -d_dot_o - t_plus_minus ) > 0) {
            t_model = t_neg;
        } else if ((t_pos = -d_dot_o + t_plus_minus) > 0) {
            t_model = t_pos;
        } else {
            return false; // all intersections are behind the ray origin
        }

        // compute point of intersection (also normal direction)
        Point3D p_intersect = origin_model + (t_model * dir_model);
        Vector3D normal = Vector3D(p_intersect);

        // transform intersection information into world space
        p_intersect = modelToWorld * p_intersect;
        normal = transNorm(worldToModel, normal);

        // the t-value for intersections in world space
        double t_world = (p_intersect - ray.origin)[0] / ray.dir[0];

        // check against prior intersection
        if (!ray.intersection.none && t_world >= ray.intersection.t_value) {
            return false;
        }

        // fill in intersection struct
        ray.intersection.none = false;
        ray.intersection.t_value = t_world;
        ray.intersection.point = p_intersect;
        ray.intersection.normal = normal;
        return true;
    }
}
