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

#ifndef CGOGN_CORE_TYPES_ANIMATION_SKELETON_H_
#define CGOGN_CORE_TYPES_ANIMATION_SKELETON_H_

#include <cgogn/core/types/container/attribute_container.h>
#include <cgogn/core/types/container/chunk_array.h>
#include <cgogn/core/functions/mesh_info.h>
#include <cgogn/core/utils/thread_pool.h>
#include <cgogn/core/utils/type_traits.h>
#include <cgogn/core/utils/numerics.h>

namespace cgogn
{

struct AnimationSkeleton
{
	static constexpr const uint8 dimension = 1;

	using AttributeContainer = AttributeContainerT<ChunkArray>;

	template <typename T>
	using Attribute = AttributeContainer::Attribute<T>;
	using AttributeGen = AttributeContainer::AttributeGen;
	using MarkAttribute = AttributeContainer::MarkAttribute;

	/*************************************************************************/
	// Cells
	/*************************************************************************/

	struct Cell
	{
		uint32 index_;
		inline Cell() : index_(INVALID_INDEX)
		{
		}
		inline Cell(uint32 id) : index_(id)
		{
		}
		operator uint32() const
		{
			return index_;
		}
		template <typename T>
		bool operator<(T other) const
		{
			return index_ < static_cast<uint32>(other);
		}
		template <typename T>
		bool operator==(T other) const
		{
			return index_ == static_cast<uint32>(other);
		}
		template <typename T>
		bool operator!=(T other) const
		{
			return index_ != static_cast<uint32>(other);
		}
		inline bool is_valid() const
		{
			return index_ != INVALID_INDEX;
		}
	};

	struct Joint : public Cell
	{
		static const uint32 CELL_INDEX = 0;
		inline Joint() : Cell() {}
		inline Joint(uint32 id) : Cell(id) {}
	};

	struct Bone : public Cell
	{
		static const uint32 CELL_INDEX = 1;
		inline Bone() : Cell() {}
		inline Bone(uint32 id) : Cell(id) {}
	};

	using Cells = std::tuple<Joint, Bone>;

	AnimationSkeleton()
	{
		bone_parent_ =
			attribute_containers_[Bone::CELL_INDEX].add_attribute<Bone>("bone_parents");
		bone_joints_ =
			attribute_containers_[Bone::CELL_INDEX].add_attribute<std::pair<Joint, Joint>>("bone_joints");
		bone_name_ =
			attribute_containers_[Bone::CELL_INDEX].add_attribute<std::string>("bone_names");
	}

	auto nb_bones() const { return bone_traverser_.size(); }

	// The vector containing all bones, with any parent *before* its children
	// This order is automatically achieved through having to specify
	// a parent on bone creation, which then doesn't change
	std::vector<Bone> bone_traverser_;

	/*************************************************************************/
	// Cells attributes containers
	/*************************************************************************/
	std::array<AttributeContainer, 2> attribute_containers_;

	std::shared_ptr<Attribute<Bone>> bone_parent_;
	std::shared_ptr<Attribute<std::pair<Joint, Joint>>> bone_joints_;
	std::shared_ptr<Attribute<std::string>> bone_name_;
};

template <typename MESH>
struct mesh_traits;

template <>
struct mesh_traits<AnimationSkeleton>
{
	static constexpr const char* name = "AnimationSkeleton";
	static constexpr const uint8 dimension = AnimationSkeleton::dimension;

	using Vertex = AnimationSkeleton::Joint;
	using Edge = AnimationSkeleton::Bone;

	using Cells = AnimationSkeleton::Cells;
	static constexpr const char* cell_names[] = {"Joint", "Bone"};

	template <typename T>
	using Attribute = AnimationSkeleton::Attribute<T>;
	using AttributeGen = AnimationSkeleton::AttributeGen;
	using MarkAttribute = AnimationSkeleton::MarkAttribute;
};

/*************************************************************************/
// Operators
/*************************************************************************/

AnimationSkeleton::Bone add_root(AnimationSkeleton& as);
AnimationSkeleton::Bone add_root(AnimationSkeleton& as, const std::string& name);
AnimationSkeleton::Bone add_root(AnimationSkeleton& as, std::string&& name);
AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent);
AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent, const std::string& name);
AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent, std::string&& name);
AnimationSkeleton::Joint get_base_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone);
AnimationSkeleton::Joint get_tip_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone);
AnimationSkeleton::Joint get_root_joint(const AnimationSkeleton& as);

/*************************************************************************/
// Cells basic functions
/*************************************************************************/

template <typename CELL>
CELL add_cell(AnimationSkeleton& as)
{
	return as.attribute_containers_[CELL::CELL_INDEX].new_index();
}

/*************************************************************************/
// Global cells traversals
/*************************************************************************/

template <typename FUNC>
auto foreach_cell(const AnimationSkeleton& as, const FUNC& f)
{
	using CELL = func_parameter_type<FUNC>;
	static_assert(has_cell_type_v<AnimationSkeleton, CELL>, "CELL not supported in this MESH");
	static_assert(is_func_return_same<FUNC, bool>::value, "Given function should return a bool");

	if constexpr (std::is_same_v<CELL, AnimationSkeleton::Joint>)
		if (as.nb_bones() == 0 || !f(get_root_joint(as)))
			return;

	for (const auto& id : as.bone_traverser_)
	{
		CELL c;

		if constexpr (std::is_same_v<CELL, AnimationSkeleton::Joint>)
			c = get_tip_joint(as, id);
		else
			c = id;

		if (!f(c))
			break;
	}
}

template <typename FUNC>
auto parallel_foreach_cell(const AnimationSkeleton& as, const FUNC& f)
{
	using CELL = func_parameter_type<FUNC>;
	static_assert(has_cell_type_v<AnimationSkeleton, CELL>, "CELL not supported in this MESH");
	static_assert(is_func_return_same<FUNC, bool>::value, "Given function should return a bool");

	ThreadPool* pool = thread_pool();
	uint32 nb_workers = pool->nb_workers();
	if (nb_workers == 0)
		return foreach_cell(as, f);

	using VecCell = std::vector<uint32>;
	using Future = std::future<void>;

	std::array<std::vector<VecCell*>, 2> cells_buffers;
	std::array<std::vector<Future>, 2> futures;
	cells_buffers[0].reserve(nb_workers);
	cells_buffers[1].reserve(nb_workers);
	futures[0].reserve(nb_workers);
	futures[1].reserve(nb_workers);

	Buffers<uint32>* buffers = uint32_buffers();

	auto it = as.bone_traverser_.cbegin();
	auto last = as.bone_traverser_.cend();

	uint32 i = 0u; // buffer id (0/1)
	uint32 j = 0u; // thread id (0..nb_workers)

	while (it < last)
	{
		// fill buffer
		cells_buffers[i].push_back(buffers->buffer());
		VecCell& cells = *cells_buffers[i].back();
		cells.reserve(PARALLEL_BUFFER_SIZE);
		for (uint32 k = 0u; k < PARALLEL_BUFFER_SIZE && it < last; ++k, ++it)
			cells.push_back(*it);
		// launch thread
		futures[i].push_back(pool->enqueue([&cells, &f]() {
			for (uint32 index : cells)
				f(CELL(index));
		}));
		// next thread
		if (++j == nb_workers)
		{ // again from 0 & change buffer
			j = 0u;
			i = (i + 1u) % 2u;
			for (auto& fu : futures[i])
				fu.wait();
			for (auto& b : cells_buffers[i])
				buffers->release_buffer(b);
			futures[i].clear();
			cells_buffers[i].clear();
		}
	}

	// clean all at the end
	for (auto& fu : futures[0u])
		fu.wait();
	for (auto& b : cells_buffers[0u])
		buffers->release_buffer(b);
	for (auto& fu : futures[1u])
		fu.wait();
	for (auto& b : cells_buffers[1u])
		buffers->release_buffer(b);
}

/*************************************************************************/
// Applicative utility
/*************************************************************************/

template <typename TransformT>
void compute_world_transforms(const AnimationSkeleton& as,
		const AnimationSkeleton::Attribute<TransformT>& local_transforms,
		AnimationSkeleton::Attribute<TransformT>& world_transforms)
{
	for (const auto& bone : as.bone_traverser_)
	{
		const auto& parent = (*as.bone_parent_)[bone];

		world_transforms[bone] = parent.is_valid()
				? world_transforms[parent] * local_transforms[bone] // non-root
				: local_transforms[bone]; // root
	}
}

} // namespace cgogn

#endif // CGOGN_CORE_TYPES_ANIMATION_SKELETON_H_
