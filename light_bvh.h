//
// Created by Rafayel Gardishyan on 17/05/2025.
//

#ifndef LIGHT_BVH_H
#define LIGHT_BVH_H
#include "hit_info.hpp"
#include "triangular_light.hpp"
#include "world.hpp"
#include "glm/vec3.hpp"

#ifndef EMBREE_H_
#define EMBREE_H_
#include <embree4/rtcore.h>
#endif


struct LightBVHPrim {
    int light_id;
    glm::vec3 position;
    float radius;
    glm::vec3 max;
    glm::vec3 min;
};

struct LightBVHPrimBox {
    glm::vec3 min;
    glm::vec3 max;

    std::vector<int> prim_ids; // Indices of LightBVHPrim in the original prims vector
};

void save_aabbs_to_obj(const std::vector<LightBVHPrim>& prims, const std::string& filename);
void save_combined_aabbs_to_obj(const std::vector<LightBVHPrimBox>& prims, const std::string& filename);

class light_bvh {
public:
    light_bvh(std::vector<TriangularLight>& lights, World& scene);

    void sample_light(HitInfo& hit, std::vector<int>& results);
private:
    std::vector<LightBVHPrim> prims;
    std::vector<LightBVHPrimBox> combined_prims;

    void _build_prims(std::vector<TriangularLight>& lights);
    void _build_prims(std::vector<TriangularLight>& lights, World& scene);

    void _combine_neighboring_prims();

    void _optimize_prims();

    // Embree
    RTCDevice device{};
    RTCScene scene{};
    RTCGeometry geom{};

    void _init_embree();

    // static C-style wrapper:
    static void boundsCallback(const RTCBoundsFunctionArguments* args) {
        // recover the instance:
        auto* self = static_cast<light_bvh*>(args->geometryUserPtr);
        self->bounds_func(args);
    }

    static void intersectCallback(const RTCIntersectFunctionNArguments* args) {
        auto* self = static_cast<light_bvh*>(args->geometryUserPtr);
        self->intersect_func(args);
    }

    static bool pointQueryFunction(RTCPointQueryFunctionArguments* args) {
        auto* hits = reinterpret_cast<std::vector<unsigned>*>(args->userPtr);
        hits->emplace_back(args->primID);
        return true; // continue querying
    }

    static void bounds_func(const struct RTCBoundsFunctionArguments* args);
    static void intersect_func(const struct RTCIntersectFunctionNArguments* args);
};

#endif //LIGHT_BVH_H
