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

#include <cgogn/core/types/animation_skeleton/animation_skeleton.h>

namespace cgogn
{

/*************************************************************************/
// Operators
/*************************************************************************/

AnimationSkeleton::Bone _internal_add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent)
{
	using Joint = AnimationSkeleton::Joint;
	using Bone = AnimationSkeleton::Bone;

	const bool& is_root = parent == INVALID_INDEX;

	cgogn_assert(is_root ? as.nb_bones() == 0 : parent < as.nb_bones());

	Joint first_joint = is_root ? add_cell<Joint>(as) : (*as.bone_joints_)[parent].second;
	Joint second_joint = add_cell<Joint>(as);
	Bone bone = add_cell<Bone>(as);

	(*as.bone_parent_)[bone] = parent;
	(*as.bone_joints_)[bone] = {first_joint, second_joint};
	(*as.bone_name_)[bone].clear();
	as.bone_traverser_.push_back(bone);

	return bone;
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

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent)
{
	return _internal_add_bone(as, parent);
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent, const std::string& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = name;
	return bone;
}

AnimationSkeleton::Bone add_bone(AnimationSkeleton& as, AnimationSkeleton::Bone parent, std::string&& name)
{
	auto bone = add_bone(as, parent);
	(*as.bone_name_)[bone] = std::move(name);
	return bone;
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

AnimationSkeleton::Joint get_root_joint(const AnimationSkeleton& as) { return get_base_joint(as, 0); }

} // namespace cgogn
