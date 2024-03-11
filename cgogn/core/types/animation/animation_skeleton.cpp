/*******************************************************************************
 * CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
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

#include <cgogn/core/types/animation/animation_skeleton.h>

namespace cgogn
{

/*************************************************************************/
// Operators
/*************************************************************************/

bool is_root(const AnimationSkeleton& as, const AnimationSkeleton::Joint& joint)
{
	for (const auto& bone : as.bone_traverser_)
		if ((*as.bone_joints_)[bone].first == joint && is_root(as, bone))
			return true;

	return false;
}

bool is_root(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	return (*as.bone_parent_)[bone] == INVALID_INDEX;
}

AnimationSkeleton::Bone add_bone_from_existing_joints(
		AnimationSkeleton& as, const AnimationSkeleton::Bone& parent,
		std::pair<AnimationSkeleton::Joint, AnimationSkeleton::Joint> joints)
{
	AnimationSkeleton::Bone bone = add_cell<AnimationSkeleton::Bone>(as);

	(*as.bone_parent_)[bone] = parent;
	(*as.bone_joints_)[bone] = joints;
	(*as.bone_name_)[bone].clear();
	as.bone_traverser_.push_back(bone);

	return bone;
}

AnimationSkeleton::Bone _internal_add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent)
{
	using Joint = AnimationSkeleton::Joint;

	const bool& is_root = parent == INVALID_INDEX;

	cgogn_assert(is_root || std::find(as.bone_traverser_.cbegin(), as.bone_traverser_.cend(), parent)
			!= as.bone_traverser_.cend());

	Joint first_joint = is_root ? add_cell<Joint>(as) : (*as.bone_joints_)[parent].second;
	Joint second_joint = add_cell<Joint>(as);

	return add_bone_from_existing_joints(as, parent, {first_joint, second_joint});
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as)
{
	return _internal_add_bone(as, INVALID_INDEX);
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as, const std::string& name)
{
	auto bone = add_root(as);
	(*as.bone_name_)[bone] = name;
	return bone;
}

AnimationSkeleton::Bone add_root(AnimationSkeleton& as, std::string&& name)
{
	auto bone = add_root(as);
	(*as.bone_name_)[bone] = std::move(name);
	return bone;
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent)
{
	return _internal_add_bone(as, parent);
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent, const std::string& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = name;
	return bone;
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, const AnimationSkeleton::Bone& parent, std::string&& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = std::move(name);
	return bone;
}

AnimationSkeleton::Bone get_parent_bone(const AnimationSkeleton& as, const AnimationSkeleton::Joint& joint)
{
	cgogn_assert(as.nb_bones() > 0);

	// Joint may be base joint of its *root* parent bone, or tip joint of its parent bone
	for (const auto& bone : as.bone_traverser_)
	{
		const auto& [first_joint, second_joint] = (*as.bone_joints_)[bone];
		if (first_joint == joint && is_root(as, bone) || second_joint == joint)
			return bone;
	}

	return INVALID_INDEX; // called with non-existent joint, or topology is broken
}

AnimationSkeleton::Joint get_base_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	cgogn_assert(bone < as.nb_bones());
	return (*as.bone_joints_)[bone].first;
}

AnimationSkeleton::Joint get_tip_joint(const AnimationSkeleton& as, const AnimationSkeleton::Bone& bone)
{
	cgogn_assert(bone < as.nb_bones());
	return (*as.bone_joints_)[bone].second;
}

void copy(AnimationSkeleton& dst, const AnimationSkeleton& src)
{
	dst = src;
}

} // namespace cgogn
