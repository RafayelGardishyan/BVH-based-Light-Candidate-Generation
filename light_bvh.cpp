//
// Created by Rafayel Gardishyan on 17/05/2025.
//

#include "light_bvh.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <ostream>
#include <glm/gtc/random.hpp>

#include "append_only_frame_time_writer.h"
#include "constants.hpp"


light_bvh::light_bvh(std::vector<TriangularLight> &lights, World &scene) {
    std::clog << "Building light BVH" << std::endl;
    prims = {};
    if (BVH_VISIBILITY_AWARE) {
        _build_prims(lights, scene);
    } else {
        _build_prims(lights);
    }
    _combine_neighboring_prims();

    if (!BVH_VISIBILITY_AWARE) {
        _optimize_prims();
    }

    _init_embree();
    std::clog << "Light BVH built" << std::endl;
    save_aabbs_to_obj(prims, AABB_SAVE_FILE);
    save_combined_aabbs_to_obj(combined_prims, "aabbs_combined.obj");
}

void light_bvh::sample_light(HitInfo &hit, std::vector<int> &results) {
    std::vector<unsigned> e_results;
    glm::vec3 hp = hit.r.at(hit.t);
    RTCPointQuery query;
    query.x = hp.x;
    query.y = hp.y;
    query.z = hp.z;
    query.radius = 0.f;

    RTCPointQueryContext pqContext;
    rtcInitPointQueryContext(&pqContext);

    rtcPointQuery(scene, &query, &pqContext, pointQueryFunction, &e_results);

    for (auto &e_result: e_results) {
        // Get the LightBVHPrim
        LightBVHPrimBox &prim = combined_prims[e_result];
        results.insert(results.end(), prim.prim_ids.begin(), prim.prim_ids.end());
    }
}

void light_bvh::_build_prims(std::vector<TriangularLight> &lights) {
    // Track time
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < lights.size(); i++) {
        TriangularLight light = lights[i];
        LightBVHPrim prim;
        prim.light_id = i;
        prim.position = (light.triangle.v0.position + light.triangle.v1.position + light.triangle.v2.position) /
                        glm::vec3(3);
        prim.radius = RADIUS_SCALING * std::sqrt(light.intensity * std::max(light.area(), 1.0f));
        prim.max = prim.position + glm::vec3(prim.radius);
        prim.min = prim.position - glm::vec3(prim.radius);
        prims.push_back(prim);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    append_only_frame_time_writer::write_frame_time(BVH_TIME_FILE, SamplingMode::BVH_Simple, duration);
    // Write timedelta to file
}

void light_bvh::_build_prims(std::vector<TriangularLight> &lights, World &scene) {
    // Build primitives with visibility awareness
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < lights.size(); i++) {
        TriangularLight light = lights[i];
        LightBVHPrim prim;
        prim.light_id = i;
        prim.position = (light.triangle.v0.position + light.triangle.v1.position + light.triangle.v2.position) /
                        glm::vec3(3);
        prim.radius = RADIUS_SCALING * std::sqrt(light.intensity * std::max(light.area(), 1.0f));
        prim.max = prim.min = light.triangle.v0.position;

        glm::vec3 max = -FLT_MAX * glm::vec3(1.0f);
        glm::vec3 min = FLT_MAX * glm::vec3(1.0f);
        for (int j = 0; j < RAYS_PER_LIGHT; j++) {
            glm::vec3 origin = sample_on_light(light);

            glm::vec3 dir = glm::normalize(glm::sphericalRand(1.0f));
            if (glm::dot(dir, light.triangle.normal()) < 0.0f) {
                origin -= 0.001f * dir;
            } else {
                origin += 0.001f * dir;
            }

            Ray ray(origin, dir);
            HitInfo hit;
            if (!scene.intersect(ray, hit, prim.radius)) {
                hit.t = prim.radius; // If no hit, set t to radius
            }
            glm::vec3 hit_point = hit.r.at(hit.t);
            max.x = glm::max(max.x, hit_point.x);
            max.y = glm::max(max.y, hit_point.y);
            max.z = glm::max(max.z, hit_point.z);
            min.x = glm::min(min.x, hit_point.x);
            min.y = glm::min(min.y, hit_point.y);
            min.z = glm::min(min.z, hit_point.z);
        }
            prim.max = max;
            prim.min = min;
        // }
        prims.push_back(prim);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    append_only_frame_time_writer::write_frame_time(BVH_TIME_FILE, SamplingMode::BVH_VisibilityAware, duration);
    // Write timedelta to file
}

void light_bvh::_combine_neighboring_prims() {
    // Combine neighboring primitives based on their bounding boxes
    combined_prims.clear();
    if (prims.empty()) {
        return; // No primitives to combine
    }
    std::vector<bool> combined(prims.size(), false);

    for (size_t i = 0; i < prims.size(); ++i) {
        if (combined[i]) {
            continue; // Already combined
        }

        LightBVHPrimBox box;
        box.min = prims[i].min;
        box.max = prims[i].max;
        box.prim_ids.push_back(prims[i].light_id);
        combined[i] = true;

        glm::vec3 center = prims[i].position;

        for (size_t j = i + 1; j < prims.size(); ++j) {
            if (combined[j]) {
                continue; // Already combined
            }

            // Check if the bounding boxes are within the COMBINE_RADIUS
            if (glm::length(center - prims[j].position) <= COMBINE_RADIUS) {
                // Combine the two primitives
                box.min = glm::min(box.min, prims[j].min);
                box.max = glm::max(box.max, prims[j].max);
                box.prim_ids.push_back(prims[j].light_id);
                combined[j] = true;
            }
        }
        combined_prims.push_back(box);
    }
}

void light_bvh::_optimize_prims() {
    int max_overlap = 0; int min_overlap = INT_MAX;
    float scale_factor = 1.0f;
    std::clog << "Optimizing bounding boxes for overlap" << std::endl;
    do {
        // Reset overlap counts
        max_overlap = 0;
        min_overlap = INT_MAX;
        // Check for overlap between bounding boxes.
        int overlaps[prims.size()] = {0};
        for (size_t i = 0; i < combined_prims.size(); ++i) {
            for (size_t j = i + 1; j < combined_prims.size(); ++j) {
                // Check if the bounding boxes overlap
                if (combined_prims[i].min.x <= combined_prims[j].max.x &&
                    combined_prims[i].max.x >= combined_prims[j].min.x &&
                    combined_prims[i].min.y <= combined_prims[j].max.y &&
                    combined_prims[i].max.y >= combined_prims[j].min.y &&
                    combined_prims[i].min.z <= combined_prims[j].max.z &&
                    combined_prims[i].max.z >= combined_prims[j].min.z) {
                    overlaps[i]++;
                    overlaps[j]++;

                    max_overlap = std::max(max_overlap, overlaps[i]);
                    min_overlap = std::min(min_overlap, overlaps[i]);
                    max_overlap = std::max(max_overlap, overlaps[j]);
                    min_overlap = std::min(min_overlap, overlaps[j]);
                    }
            }
        }
        std::clog << "Max overlap: " << max_overlap << ", Min overlap: " << min_overlap << std::endl;
        // Ensure that every bounding box only has 5 overlaps max and 1 overlap min.
        // Scale the bounding boxes to achieve this.
        scale_factor += max_overlap > 5 ? -0.1f : 0.1f; // Adjust scale factor based on overlap
        std::clog << "Scaling factor: " << scale_factor << std::endl;
        for (size_t i = 0; i < combined_prims.size(); ++i) {

                glm::vec3 center = (combined_prims[i].min + combined_prims[i].max) / 2.0f;
                glm::vec3 size = combined_prims[i].max - combined_prims[i].min;
                size *= scale_factor;
                combined_prims[i].min = center - size / 2.0f;
                combined_prims[i].max = center + size / 2.0f;
        }
    } while (max_overlap > 5 || min_overlap < 1);
    std::clog << "Bounding boxes optimized for overlap" << std::endl;
}


void light_bvh::_init_embree() {
    device = rtcNewDevice(NULL);
    scene = rtcNewScene(device);
    geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);

    rtcSetGeometryUserPrimitiveCount(geom, combined_prims.size());
    rtcSetGeometryBoundsFunction(geom, boundsCallback, this);
    rtcSetGeometryIntersectFunction(geom, intersectCallback);
    rtcSetGeometryUserData(geom, combined_prims.data());

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcCommitScene(scene);
    rtcReleaseGeometry(geom);
}

void light_bvh::bounds_func(const struct RTCBoundsFunctionArguments *args) {
    const auto *prims = (const LightBVHPrimBox *) args->geometryUserPtr;
    const LightBVHPrimBox &prim = prims[args->primID];
    RTCBounds *bounds_o = args->bounds_o;
    bounds_o->lower_x = prim.min.x;
    bounds_o->lower_y = prim.min.y;
    bounds_o->lower_z = prim.min.z;
    bounds_o->upper_x = prim.max.x;
    bounds_o->upper_y = prim.max.y;
    bounds_o->upper_z = prim.max.z;
}

void light_bvh::intersect_func(const struct RTCIntersectFunctionNArguments *args) {
    assert(args->N == 1);
    RTCRayHit *rayhit = (RTCRayHit *) args->rayhit;
    const LightBVHPrimBox *prims = (const LightBVHPrimBox *) args->geometryUserPtr;
    const LightBVHPrimBox &prim = prims[args->primID];

    glm::vec3 o = glm::vec3(rayhit->ray.org_x, rayhit->ray.org_y, rayhit->ray.org_z);
    glm::vec3 d = glm::vec3(rayhit->ray.dir_x, rayhit->ray.dir_y, rayhit->ray.dir_z);

    // Perform the Slab AABB intersection test
    float tmin = (prim.min.x - o.x) / d.x;
    float tmax = (prim.max.x - o.x) / d.x;
    if (tmin > tmax) std::swap(tmin, tmax);
    float tymin = (prim.min.y - o.y) / d.y;
    float tymax = (prim.max.y - o.y) / d.y;
    if (tymin > tymax) std::swap(tymin, tymax);
    if (tmin > tymax || tymin > tmax) {
        return; // No intersection
    }
    tmin = std::max(tmin, tymin);
    tmax = std::min(tmax, tymax);
    float tzmin = (prim.min.z - o.z) / d.z;
    float tzmax = (prim.max.z - o.z) / d.z;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);
    if (tmin > tzmax || tzmin > tmax) {
        return; // No intersection
    }
    tmin = std::max(tmin, tzmin);
    float t = std::min(tmax, tzmax);
    if (t < rayhit->ray.tnear || t > rayhit->ray.tfar) {
        return; // No intersection within the ray bounds
    }

    rayhit->ray.tfar = t;
    rayhit->hit.geomID = args->geomID;
    rayhit->hit.primID = args->primID;
}

void save_aabbs_to_obj(const std::vector<LightBVHPrim> &prims, const std::string &filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    file << "# AABBs from light BVH\n";
    int vertexOffset = 1;
    for (const auto &prim: prims) {
        const glm::vec3 &min = prim.min;
        const glm::vec3 &max = prim.max;

        // 8 vertices of the AABB
        glm::vec3 v[8] = {
            {min.x, min.y, min.z},
            {max.x, min.y, min.z},
            {max.x, max.y, min.z},
            {min.x, max.y, min.z},
            {min.x, min.y, max.z},
            {max.x, min.y, max.z},
            {max.x, max.y, max.z},
            {min.x, max.y, max.z}
        };

        // Write vertices
        for (int i = 0; i < 8; ++i)
            file << "v " << v[i].x << " " << v[i].y << " " << v[i].z << "\n";

        // Define the 12 edges of the box using lines
        const int edges[12][2] = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top
            {0, 4}, {1, 5}, {2, 6}, {3, 7} // sides
        };

        for (const auto &edge: edges)
            file << "l " << (vertexOffset + edge[0]) << " " << (vertexOffset + edge[1]) << "\n";

        vertexOffset += 8;
    }
    file.close();
    std::cout << "AABBs saved to " << filename << std::endl;
}

void save_combined_aabbs_to_obj(const std::vector<LightBVHPrimBox> &prims, const std::string &filename) {
    std::vector<LightBVHPrim> cp;
    for (const auto &prim: prims) {
        cp.push_back({
            0, glm::vec3(0.0f), 0.f, prim.max, prim.min
        });
    }
    save_aabbs_to_obj(cp, filename);
}
