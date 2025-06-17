#include "light_sampler.hpp"

#include <glm/vec3.hpp>
#include <vector>
#include <array>
#include <span>
#include <random>
#include <algorithm>

#include "constants.hpp"
#include "world.hpp"
#include "ray.hpp"
#include "triangular_light.hpp"
#include "hit_info.hpp"


thread_local std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<float> dist(0.0f, 1.0f);

SampleInfo::SampleInfo() : light(glm::vec3(0.0), glm::vec3(0.0), glm::vec3(0.0), glm::vec3(0.0), 0), light_point(0.0f) {
}

SampleInfo::SampleInfo(const TriangularLight &light, const glm::vec3 &light_point) : light(light),
    light_point(light_point) {
}

SampleInfo::SampleInfo(const TriangularLight &light, const glm::vec3 &light_point, const int num_found,
                       const bool found_with_bvh) : light(light),
                                                    light_point(light_point), num_lights_found(num_found),
                                                    found_with_bvh(found_with_bvh) {
}

Reservoir::Reservoir() : M(0),
                         w_sum(0.0f), phat(0.0f), y(), W(0.0f) {
}

SamplerResult::SamplerResult() : light_point(0.0f), light_dir(0.0f),
                                 light(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f), 0), W(0.0f) {
}


bool Reservoir::update(const SampleInfo x_i, const float w_i, const float n_phat) {
    w_sum = w_sum + w_i;
    M = M + 1;
    // Condition for when w_i is 0 and w_sum is also 0 so we get 0/0
    if (dist(rng) < w_i / w_sum) {
        y = x_i;
        phat = n_phat;
        return true;
    }
    return false;
}

static float calculate_reservoir_weight(const float phat, const float M, const float w_sum) {
    if (phat >= 0.0001f) {
        return (1.0f / phat) * ((1.0f / M) * w_sum);
    } else {
        return 0.0f;
    }
}

Reservoir Reservoir::merge(const Reservoir &r1, const Reservoir &r2) {
    // Merge the other reservoir into this one
    Reservoir s;

    s.update(r1.y, r1.phat * r1.W * r1.M, r1.phat);
    s.update(r2.y, r2.phat * r2.W * r2.M, r2.phat);

    s.M = r1.M + r2.M;

    s.W = calculate_reservoir_weight(s.phat, s.M, s.w_sum);

    return s;
}

Reservoir Reservoir::combineReservoirsUnbiased(const std::span<const Reservoir *> &reservoirs) {
    Reservoir s;
    for (auto &r: reservoirs) {
        s.update(r->y, r->phat * r->W * r->M, r->phat);
    }

    s.M = 0;
    for (auto &r: reservoirs) {
        s.M += r->M;
    }

    int Z = 0;

    for (auto &r: reservoirs) {
        if (r->phat > 0) {
            Z += r->M;
        }
    }

    s.W = calculate_reservoir_weight(s.phat, Z, s.w_sum);

    return s;
}

void Reservoir::replace(const Reservoir &other) {
    W = other.W;
    M = other.M;
    y = other.y;
    w_sum = other.w_sum;
    phat = other.phat;
}

Reservoir Reservoir::combineReservoirs(const std::span<const Reservoir *> &reservoirs) {
    Reservoir s;
    for (auto &r: reservoirs) {
        s.update(r->y, r->phat * r->W * r->M, r->phat);
    }

    s.M = 0;
    for (auto &r: reservoirs) {
        s.M += r->M;
    }

    s.W = calculate_reservoir_weight(s.phat, s.M, s.w_sum);

    return s;
}

void Reservoir::reset() {
    y = SampleInfo();
    M = 0;
    W = 0.0f;
    w_sum = 0.0f;
    phat = 0.0f;
}

RestirLightSampler::RestirLightSampler(const int x, const int y,
                                       std::vector<TriangularLight> &lights_vec, World &scene) : x_pixels(x),
    y_pixels(y),
    lbvh(lights_vec, scene) {
    prev_reservoirs = std::vector(y * x, Reservoir());
    current_reservoirs = std::vector(y * x, Reservoir());
    num_lights = static_cast<int>(lights_vec.size());
    lights = lights_vec.data();
}

void RestirLightSampler::reset() {
    // Reset the reservoirs
    for (int y = 0; y < y_pixels; y++) {
        for (int x = 0; x < x_pixels; x++) {
            current_reservoirs[y * x_pixels + x].reset();
            prev_reservoirs[y * x_pixels + x].reset();
        }
    }
}

std::vector<std::vector<SamplerResult> >
RestirLightSampler::sample_lights(std::vector<HitInfo> hit_infos, World &scene) {
    if (num_lights == 0) {
        return std::vector(y_pixels, std::vector<SamplerResult>(x_pixels));
    }
    // Swap the current and previous reservoirs: The previous current frame is now the previous frame
    swap_buffers();

    // For every pixel:
    // 1. Sample M times from the light sources; Choose one sample (reservoir)
    // 2. Check visibility of the light sample
    // 3. Temporal update - update the current reservoir with the previous one
    // 4. Spatial update - update the current reservoir with the neighbors
    // 5. Return the sample in the current reservoir

#pragma omp parallel for
    for (int y = 0; y < y_pixels; y++) {
        for (int x = 0; x < x_pixels; x++) {
            HitInfo &hi = hit_infos[y * x_pixels + x];

            if (hi.t == 1E30f || hi.mat_ptr->emits_light()) {
                continue;
            }

            Reservoir &current = current_reservoirs[y * x_pixels + x];
            current.reset();

            Reservoir &prev = prev_reservoirs[y * x_pixels + x];

            set_initial_sample(current, hi);
            visibility_check(current, hi, scene);

            if (sampling_mode != SamplingMode::Uniform && sampling_mode != SamplingMode::RIS) {
                prev.M = fmin(M_CAP * current.M, prev.M);
                current_reservoirs[y * x_pixels + x] = temporal_update(current, prev);
            }
        }
    }

    std::vector results(y_pixels, std::vector<SamplerResult>(x_pixels));
    if (sampling_mode != SamplingMode::Uniform && sampling_mode != SamplingMode::RIS) {
        swap_buffers();
    }
#pragma omp parallel for
    for (int y = 0; y < y_pixels; y++) {
        for (int x = 0; x < x_pixels; x++) {
            if (sampling_mode != SamplingMode::Uniform && sampling_mode != SamplingMode::RIS) {
                spatial_update(x, y, hit_infos, scene);
            }

            Reservoir &res = current_reservoirs[y * x_pixels + x];
            HitInfo &hi = hit_infos[y * x_pixels + x];

            results[y][x].light_point = res.y.light_point;
            results[y][x].light_dir = normalize(res.y.light_point - hi.r.at(hi.t));
            results[y][x].light = res.y.light;
            results[y][x].W = res.W;
        }
    }
    return results;
}

void RestirLightSampler::set_initial_sample(Reservoir &r, HitInfo &hi) {
    // Sample M times from the light sources
    std::vector<SampleInfo> samples;

    // find relevant lights if sampling from BVH
    std::vector<int> found_lights;
    if (sampling_mode == SamplingMode::ReSTIRBVH)
        lbvh.sample_light(hi, found_lights);

    int num_found = static_cast<int>(found_lights.size());
    int u = 0, b = 0;
    if (sampling_mode == SamplingMode::Uniform) {
        TriangularLight l = pick_light();
        const glm::vec3 sample_point(sample_on_light(l));
        samples.emplace_back(SampleInfo(l, sample_point, num_lights));
    } else {
#pragma omp parallel for
        for (int k = 0; k < m; k++) {
            int index = 0;

            int choose_num = num_lights;

            // If we sample from bvh
            if (sampling_mode == SamplingMode::ReSTIRBVH && (k % INTERLEAVE_EVERY_N) != 0 && num_found > 0) {
                // Interleave with uniform sampling
                if (num_found == 0)
                    continue;
                // For the source distribution, set the number of lights to the number of found lights
                choose_num = num_found;
                // Pick a random light from the found lights
                index = found_lights[dist(rng) * num_found];
                ++b;
            } else {
                // If we sample uniformly
                // Just pick a light uniformly
                index = sampleLightIndex();
                ++u;
            }

            // Sample a point on the light
            TriangularLight l = lights[index];
            const glm::vec3 sample_point(sample_on_light(l));
            bool found_contains = std::find(found_lights.begin(), found_lights.end(), index) != found_lights.end();
            samples.emplace_back(l, sample_point, choose_num, found_contains);
        }
    }

    for (auto &sample: samples) {
        float W, phat;
        get_light_weight(sample, hi, W, phat, u, b);
        r.update(sample, W, phat);

        if (sampling_mode == SamplingMode::Uniform)
            break;
        r.W = (1.0f / r.phat) * ((1.0f / r.M) * r.w_sum);
    }
}


void RestirLightSampler::visibility_check(Reservoir &res, const HitInfo &hi, World &scene, bool reset_phat) {
    // Check the visibility of the light sample

    // Point of intersection [x]
    const glm::vec3 I = hi.r.at(hi.t);

    // Distance between the light and the intersection point
    const float dist = glm::length(res.y.light_point - I);

    const glm::vec3 L = (res.y.light_point - hi.r.at(hi.t)) / dist; // Direction to light

    Ray shadow_ray = Ray(I + 0.001f * L, L);
    const bool is_occluded = scene.is_occluded(shadow_ray, dist - 0.1f);

    if (is_occluded) {
        res.W = 0;
        if (reset_phat) {
            res.phat = 0;
        }
    }
}

Reservoir RestirLightSampler::temporal_update(const Reservoir &current, const Reservoir &prev) {
    std::array<const Reservoir *, 2> refs = {&current, &prev};
    return Reservoir::combineReservoirs(refs);
}

void RestirLightSampler::spatial_update(const int x, const int y, const std::vector<HitInfo> &hit_infos, World &scene) {
    int c = 0;

    std::vector<const Reservoir *> candidates;

    const HitInfo &current_hit = hit_infos[y * x_pixels + x];

    const glm::vec3 N = current_hit.triangle.normal(current_hit.uv);

    // candidates.push_back(&current_reservoirs[y * x_pixels + x]);

    // Generate neighbours by randomly sampling a NEIGHBOUR_RADIUS radius around the current pixel
    std::array<glm::ivec2, NEIGHBOUR_K> offsets;

    for (int i = 0; i < NEIGHBOUR_K; i++) {
        const float phi = dist(rng) * 2.0f * glm::pi<float>();
        const float r = dist(rng) * NEIGHBOUR_RADIUS;

        const float x_offset = r * cosf(phi);
        const float y_offset = r * sinf(phi);

        offsets[i] = glm::ivec2(x_offset, y_offset);
    }

    for (glm::ivec2 &offset: offsets) {
        const int nx = x + offset.x;
        const int ny = y + offset.y;

        const bool x_within_bounds = nx >= 0 && nx < x_pixels;
        const bool y_within_bounds = ny >= 0 && ny < y_pixels;

        if (x_within_bounds && y_within_bounds) {
            const HitInfo &hi = hit_infos[ny * x_pixels + nx];

            // If the neighbour has no hit or is a light source, skip it, since it's reservoir is not valid
            const bool invalid_sample = hi.t == 1E30f || hi.mat_ptr->emits_light();

            // Check if the normals are similar
            const glm::vec3 N2 = hi.triangle.normal(hi.uv);
            const bool different_normals = glm::distance(N, N2) > NORMAL_DEVIATION;

            // Check if the hits are not far away from each other
            const float dist = glm::distance(current_hit.r.at(current_hit.t), hi.r.at(hi.t));
            const bool different_t = dist > T_DEVIATION;

            Reservoir &candidate = prev_reservoirs[ny * x_pixels + nx];

            if (!invalid_sample && !different_normals && !different_t) {
                candidates.push_back(&candidate);

                // Visibility check for everything except current pixel since that was already done before the temporal reuse
                if (offset.x != 0 && offset.y != 0) {
                    visibility_check(candidate, hi, scene, true);
                }
            }
        }
    }

    current_reservoirs[y * x_pixels + x] = Reservoir::combineReservoirs(std::span(candidates));
}

void RestirLightSampler::swap_buffers() {
    // Swap the current and previous reservoirs
    current_reservoirs.swap(prev_reservoirs);
}

[[nodiscard]] TriangularLight RestirLightSampler::pick_light() const {
    // Pick a random light source uniformly (standard, change this if you want to use a different sampling strategy)
    const int index = sampleLightIndex();
    return lights[index];
}

int RestirLightSampler::sampleLightIndex() const {
    // Thread-local pair to track last used upper bound and distribution
    thread_local int cached_num_lights = -1;

    if (cached_num_lights != num_lights) {
        cached_num_lights = num_lights;
    }

    const int out = static_cast<int>(dist(rng) * cached_num_lights);

    return out;
}

static float luminance(const glm::vec3 &color) {
    return 0.2126f * color.r + 0.7152f * color.g + 0.0722f * color.b;
}

float RestirLightSampler::get_mis_pdf(int num_found,
                                      bool found_with_bvh,
                                      float light_area,
                                      float dist2,
                                      float cos_theta_light,
                                      int u, int b) const {
    auto n2 = static_cast<float>(u);
    auto n1 = static_cast<float>(b);
    const float inv_area = 1.0f / light_area;

    // Strategy PDFs in area measure
    float p1 = found_with_bvh
                   ? inv_area / static_cast<float>(num_found)
                   : 0.0f;
    float p2 = inv_area / static_cast<float>(num_lights);

    // Balance‐heuristic mixture in area
    float p_area = (n1 * p1 + n2 * p2) / (n1 + n2);

    // Convert to solid angle
    return p_area * (dist2 / cos_theta_light);
}

float RestirLightSampler::get_source_pdf(int num_lights, float light_area, float dist2, float cos_theta_light) const {
    const float light_choose_pdf = 1.0f / static_cast<float>(num_lights);
    const float light_area_pdf = 1.0f / light_area;

    return light_choose_pdf * light_area_pdf * (dist2 / cos_theta_light); // dA → dOmega
}

void RestirLightSampler::get_light_weight(const SampleInfo &sample,
                                          const HitInfo &hi, float &W, float &phat, int u, int b) const {
    // Geometry setup
    const glm::vec3 hit_point = hi.r.at(hi.t);
    const glm::vec3 light_vec = sample.light_point - hit_point;
    const float dist2 = glm::dot(light_vec, light_vec);
    const glm::vec3 L = glm::normalize(light_vec); // Direction to light

    const glm::vec3 N = hi.triangle.normal(hi.uv); // Surface normal
    const glm::vec3 Nl = sample.light.triangle.normal(); // Light normal

    const float cos_theta = fabs(glm::dot(N, L)); // Surface angle
    const float cos_theta_light = fabs(glm::dot(Nl, -L)); // Light angle

    // BRDF
    const glm::vec3 fr = hi.mat_ptr->evaluate(hi, L); // f_r
    const glm::vec3 Le = sample.light.intensity * sample.light.c; // L_i

    // Target importance (importance of this sample for the current pixel)
    const float G = cos_theta;
    const float target = luminance(Le * fr * G);

    float source;
    // Source PDF: converting from area to solid angle
    if (sampling_mode == SamplingMode::ReSTIRBVH) {
        source = get_mis_pdf(sample.num_lights_found, sample.found_with_bvh, sample.light.area(), dist2,
                             cos_theta_light, u, b);
    } else {
        source = get_source_pdf(num_lights, sample.light.area(), dist2, cos_theta_light);
    }

    // Final weight and importance
    W = target / source;

    phat = target; // This is the "target pdf" value used by ReSTIR
}
