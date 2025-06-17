#pragma once

#ifndef TINY_BVH_H_
#include "lib/tiny_bvh.h"
#endif
#ifndef TINY_OBJ_LOADER_H_
#include "lib/tiny_obj_loader.h"
#endif
#ifndef EMBREE_H_
#include <embree4/rtcore.h>
#endif


#include <string>
#include <vector>

#include "triangular_light.hpp"
#include "material.hpp"

class World
{
	public:
	std::vector<Triangle> triangle_soup;
	std::vector<tinyobj::material_t> all_materials;
	std::vector<Triangle> lights;
	std::vector<tinyobj::material_t> light_materials;
	
	
	World(); // constructor makes an empty world

	void add_obj(std::string file, bool is_lights); // Add an obj, indicate if it is all lights
	void place_obj(std::string file, bool is_lights, glm::vec3 position);

	bool intersect(Ray& ray, HitInfo& hit, float max_t = 1E30f);
	bool is_occluded(const Ray &ray, float dist);

	tinybvh::BVH& bvh(); // Build the bvh


	std::vector<TriangularLight> get_triangular_lights();
	std::vector<Material*> get_materials(bool ignore_textures = true);


	private:
	std::vector<int> all_material_ids;
	std::vector<int> light_material_ids;
	std::vector<Material*> mats_small;
	tinybvh::BVH bvhInstance;
	bool bvh_built = false;
	std::vector<tinybvh::bvhvec4> raw_bvh_data;

	void embree_init(); // Initialize Embree
	bool embree_intersect(Ray &r, HitInfo &hit, float max_t = std::numeric_limits<float>::infinity());
	bool embree_is_occluded(const Ray &r, float dist);

	// Embree
	RTCDevice device;
	RTCScene scene;
	RTCGeometry geom;

	void load_obj_at(std::string& file_path, glm::vec3 position, bool force_light = false);
};

World load_world();