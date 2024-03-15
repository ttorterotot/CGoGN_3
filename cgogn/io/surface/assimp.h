/*******************************************************************************
 * CGoGN                                                                        *
 * Copyright (C), IGG Group, ICube, University of Strasbourg, France            *
 *                                                                              *
 * This library is free software; you can redistribute it and/or modify it      *
 * under the terms of the GNU Lesser General Public License as published by the *
 * Free Software Foundation; either version 2.1 of the License, or (at your     *
 * option) any later version.                                                   *
 *                                                                              *
 * This library is distributed in the hope that it will be useful, but WITHOUT  *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
 * for more details.                                                            *
 *                                                                              *
 * You should have received a copy of the GNU Lesser General Public License     *
 * along with this library; if not, write to the Free Software Foundation,      *
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
 *                                                                              *
 * Web site: http://cgogn.unistra.fr/                                           *
 * Contact information: cgogn@unistra.fr                                        *
 *                                                                              *
 *******************************************************************************/

#ifndef CGOGN_MODULE_ASSIMP_H_
#define CGOGN_MODULE_ASSIMP_H_

#include <optional>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/animation/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>
#include <cgogn/io/surface/surface_import.h>

namespace cgogn
{

namespace io
{

class AssimpImporter
{
public:
	template <typename T, typename U>
	using Map = typename std::unordered_map<T, U>;

private:
	using Skeleton = AnimationSkeleton;

private:
	template <typename FUNC>
	static void visit_node(const aiScene& scene, const aiNode* node, size_t depth, const FUNC& f)
	{
		if (!node)
			return;

		f(*node, depth);

		for (size_t i = 0; i < node->mNumChildren; ++i)
			visit_node(scene, node->mChildren[i], depth + 1, f);
	}

	template <typename Surface>
	static void load(const aiScene& scene,
			Map<std::string, Surface*>& surfaces,
			std::optional<Skeleton*>& skeleton,
			bool normalized)
	{
		visit_node(scene, scene.mRootNode, 0, [](const aiNode& node, const size_t& depth)
		{
			for (size_t j = 0; j < depth; ++j)
				std::cout << "  ";
			std::cout << node.mName.C_Str() << std::endl;
		});
	}

public:
	/// @brief Opens and parses an FBX file, filling the maps with objects inside
	/// @param path the file path to open
	/// @param surfaces a map to fill with surfaces described in the file
	/// @param skeletons an optional that would be set with the animation skeleton that may be described in the file
	/// @param normalized whether or not to normalize the positions for each surface (can offset it from bones)
	template <typename Surface>
	static void load(const std::string& path,
			Map<std::string, Surface*>& surfaces,
			std::optional<Skeleton*>& skeleton,
			bool normalized = false)
	{
		Assimp::Importer importer{}; // object actually holds the data, lifetime needs to be longer than call to load
		if (auto* scene = importer.ReadFile(path, aiProcess_ValidateDataStructure))
			load(*scene, surfaces, skeleton, normalized);
		else
			skeleton = {};
	}
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_MODULE_ASSIMP_H_
