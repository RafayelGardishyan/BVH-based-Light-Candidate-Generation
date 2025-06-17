#pragma once
#include <string>
constexpr auto BASE_LIGHT_INTENSITY = 20.0f;
constexpr auto ENABLE_TEXTURES = true;
constexpr auto M_CAP = 20.0f;
constexpr auto DEFAULT_M = 32;
constexpr auto NORMAL_DEVIATION = 0.5f;
constexpr auto T_DEVIATION = 0.05f;
constexpr auto SAVE_INTERMEDIATE = true;
constexpr auto RENDER_FRAME_COUNT = 20;
constexpr auto GT_FRAME_COUNT = 4096; // Number of frames to render for ground truth
constexpr auto RENDER_WIDTH = 1920;
constexpr auto LIVE_WIDTH = 400;
constexpr auto SAVE_FORMAT = 1; // 0: png, 1: pfm
constexpr auto BVH_LIB = 0; // 0 == tinybvh; 1 == embree
constexpr auto NEIGHBOUR_K = 8;
constexpr auto NEIGHBOUR_RADIUS = 5; // pixels

inline bool BVH_VISIBILITY_AWARE = false; // Use visibility-aware BVH sampling

constexpr auto INTERLEAVE_BVH = true; // Interleave BVH Sampling with uniform to avoid bias
constexpr auto INTERLEAVE_EVERY_N = 8; // Interleave every N samples

constexpr auto AABB_SAVE_FILE = "aabbs.obj"; // File to save AABBs
constexpr auto RADIUS_SCALING = 1.0f; // Scaling factor for radius
constexpr auto RAYS_PER_LIGHT = 200; // Number of rays per light

constexpr auto COMBINE_RADIUS = .5f; // Radius to combine neighboring primitives

constexpr std::string MODEL_NAME = "monkey"; // Model to render

inline const auto FRAME_TIME_CSV = "frame_" + MODEL_NAME + ".csv"; // File to save frame times
inline const auto BVH_TIME_FILE = "bvh_" + MODEL_NAME + ".csv"; // File to save BVH build times
//#define INTERPOLATE_NORMALS true
