#pragma once

#include <glm/vec3.hpp>
#include <vector>
#include <array>
#include <span>
#include <random>
#include <iostream>

#include "constants.hpp"
#include "triangular_light.hpp"
#include "ray.hpp"
#include "world.hpp"
#include "hit_info.hpp"
#include "light_bvh.h"


enum class SamplingMode {
    ReSTIRBVH,
    ReSTIR,
    Uniform,
    RIS,

    BVH_Simple,
    BVH_VisibilityAware,
};

inline std::ostream &operator<<(std::ostream &out, const SamplingMode &mode) {
    switch (mode) {
        case SamplingMode::ReSTIRBVH:
            out << std::string("ReSTIRBVH");
            break;
        case SamplingMode::ReSTIR:
            out << std::string("ReSTIR");
            break;
        case SamplingMode::Uniform:
            out << std::string("Uniform");
            break;
        case SamplingMode::RIS:
            out << std::string("RIS");
            break;
        case SamplingMode::BVH_Simple:
            out << std::string("BVH Simple");
            break;
        case SamplingMode::BVH_VisibilityAware:
            out << std::string("BVH Visibility Aware");
    }
    return out;
}


struct SamplerResult {
    glm::vec3 light_point;
    glm::vec3 light_dir;
    TriangularLight light;
    float W;

    SamplerResult();
};

struct SampleInfo {
    TriangularLight light;
    glm::vec3 light_point;

    int num_lights_found = -1;
    bool found_with_bvh = false;

    SampleInfo();

    SampleInfo(const TriangularLight &light, const glm::vec3 &light_point);
    SampleInfo(const TriangularLight &light, const glm::vec3 &light_point, const int num_lights_found, const bool found_with_bvh = false);
};

class Reservoir {
public:
    SampleInfo y;
    float w_sum;
    int M;
    float phat;
    float W;

    Reservoir();

    bool update(const SampleInfo x_i, const float w_i, const float n_phat);

    static Reservoir merge(const Reservoir &r1, const Reservoir &r2);

    static Reservoir combineReservoirs(const std::span<const Reservoir *> &reservoirs);
    static Reservoir combineReservoirsUnbiased(const std::span<const Reservoir*>& reservoirs);

    void replace(const Reservoir &other);

    void reset();
};

class RestirLightSampler {
public:
    RestirLightSampler(const int x, const int y,
                       std::vector<TriangularLight> &lights_vec, World& scene);

    void reset();

    std::vector<std::vector<SamplerResult> > sample_lights(std::vector<HitInfo> hit_infos, World &scene);

    void set_initial_sample(Reservoir &r, HitInfo &hi);

    void visibility_check(Reservoir& res, const HitInfo& hi, World& world, bool reset_phat = false);

    Reservoir temporal_update(const Reservoir &current, const Reservoir &prev);

    void spatial_update(const int x, const int y, const std::vector<HitInfo>& hit_infos, World& scene);

    void swap_buffers();

    int m = DEFAULT_M;

    SamplingMode sampling_mode = SamplingMode::ReSTIR;

    int num_lights;

private:
    int x_pixels;
    int y_pixels;
    std::vector<Reservoir> prev_reservoirs;
    std::vector<Reservoir> current_reservoirs;
    TriangularLight *lights;
    light_bvh lbvh;

    [[nodiscard]] TriangularLight pick_light() const;

    [[nodiscard]] int sampleLightIndex() const;

    void get_light_weight(const SampleInfo &sample,
                          const HitInfo &hi, float &W, float &phat, int u, int b) const;

    float get_source_pdf(int num_lights, float light_area, float dist2, float cos_theta_light) const;
    float get_mis_pdf(int num_found, bool found_with_bvh, float light_area, float dist2, float cos_theta_light, int u, int b) const;
};
