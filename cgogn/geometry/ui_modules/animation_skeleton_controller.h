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

#ifndef CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_
#define CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_

#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/animation/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>

#define DUAL_QUATERNION_USE_SHORTEST_PATH_FOR_INTERPOLATION 1

namespace cgogn
{

namespace ui
{

using geometry::Vec3;
using geometry::Vec4;
using geometry::Scalar;

template <template <typename> typename ContainerT, typename TimeT, typename TransformT>
class AnimationSkeletonController : public Module
{
public:
	enum class CheckParams
	{
		CheckAndWarnInvalid,
		CheckInvalid,
		AssumeValid,
	};

private:
	using MESH = AnimationSkeleton;

	template <typename T>
	using Attribute = AnimationSkeleton::Attribute<T>;

	using Joint = AnimationSkeleton::Joint;
	using Bone = AnimationSkeleton::Bone;

	using AnimationT = geometry::KeyframedAnimation<ContainerT, TimeT, TransformT>;

public:
	AnimationSkeletonController(const App& app) : Module(app, "AnimationSkeletonController (" + std::string{mesh_traits<MESH>::name} + ")")
	{
	}
	~AnimationSkeletonController()
	{
	}

	/// @brief Changes the linked bone animation attribute.
	/// Does not check if a skeleton is already selected.
	/// @param attribute the new attribute to use as animations
	/// @param check_params what to check about animation validity (size and sorting)
	void set_animation(const std::shared_ptr<Attribute<AnimationT>>& attribute, CheckParams check_params = CheckParams::CheckAndWarnInvalid)
	{
		selected_animation_ = attribute;

		bool all_sorted = true;

		if (check_params != CheckParams::AssumeValid)
		{
			// Check for empty animations
			cgogn_message_assert(std::find_if(attribute->begin(), attribute->end(), [](const AnimationT& anim){ return anim.size() == 0; })
					== attribute->end(), "[AnimationSkeletonController::set_animation] Found empty animation");
			// Check if all bones' animations are sorted
			all_sorted = std::find_if_not(attribute->begin(), attribute->end(), [](const AnimationT& anim){ return anim.is_sorted(); })
					== attribute->end();
		}

		if (!all_sorted && check_params == CheckParams::CheckAndWarnInvalid)
			std::cout << "[AnimationSkeletonController::set_animation] Found unsorted animation, "
					"skeleton may be animated improperly" << std::endl;

		update_animation_time_extrema(all_sorted);
		update_embedding();
	}

	/// @brief Changes the linked joint position attribute.
	/// Does not check if a skeleton is already selected.
	/// @param attribute the new attribute to use as positions
	void set_joint_position(const std::shared_ptr<Attribute<Vec3>>& attribute)
	{
		selected_joint_position_ = attribute;
		update_joint_positions();
	}

	/// @brief Changes the current time of the animation, and updates positions accordingly if a skeleton is selected.
	/// @param time the new time to set the animation to
	void set_time(TimeT time)
	{
		time_ = time;

		if (selected_skeleton_)
			update_embedding();
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<ui::MeshProvider<MESH>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<MESH>::name} + ")"));
	}

	void left_panel() override
	{
		imgui_mesh_selector(mesh_provider_, selected_skeleton_, "Skeleton", [&](MESH& m) {
			selected_skeleton_= &m;
			selected_animation_.reset();
			selected_joint_position_ = cgogn::get_attribute<Vec3, Joint>(m, "position"); // nullptr (equiv. to reset) if not found
			selected_bone_local_transform_ = cgogn::get_or_add_attribute<TransformT, Bone>(m, "local_transform");
			selected_bone_global_transform_ = cgogn::get_or_add_attribute<TransformT, Bone>(m, "global_transform");
		});

		if (selected_skeleton_)
		{
			imgui_combo_attribute<Bone, AnimationT>(*selected_skeleton_, selected_animation_,
					"Animation", [&](const std::shared_ptr<Attribute<AnimationT>>& attribute){ set_animation(attribute); });
			imgui_combo_attribute<Joint, Vec3>(*selected_skeleton_, selected_joint_position_,
					"Position", [&](const std::shared_ptr<Attribute<Vec3>>& attribute){ set_joint_position(attribute); });
			
			if (selected_animation_)
			{
				float t = static_cast<float>(time_);
				if (ImGui::SliderFloat("Time", &t, selected_animation_start_time_, selected_animation_end_time_))
					set_time(static_cast<TimeT>(t));
			}
		}
	}

private:
	// Caches the first and last keyframe's times across all bones' animations
	// Assumes all animations have at least one keyframe
	void update_animation_time_extrema(bool assume_all_sorted)
	{
		selected_animation_start_time_ = std::numeric_limits<TimeT>::max();
		selected_animation_end_time_ = std::numeric_limits<TimeT>::lowest();

		if (assume_all_sorted)
		{
			for (const auto& anim : *selected_animation_)
			{
				selected_animation_start_time_ = std::min(selected_animation_start_time_, anim[0].time_);
				selected_animation_end_time_ = std::max(selected_animation_end_time_, anim[anim.size() - 1].time_);
			}
			return;
		}

		for (const auto& anim : *selected_animation_)
		{
			for (const auto& keyframe : anim)
			{
				selected_animation_start_time_ = std::min(selected_animation_start_time_, keyframe.time_);
				selected_animation_end_time_ = std::max(selected_animation_end_time_, keyframe.time_);
			}
		}
	}

	// Computes local/global transforms and resulting positions
	// Assumes a skeleton and animation are already selected, with corresponding transform attributes
	void update_embedding()
	{
		for (const auto& bone : selected_skeleton_->bone_traverser_)
			(*selected_bone_local_transform_)[bone] = get_current_transform((*selected_animation_)[bone]);
		compute_world_transforms(*selected_skeleton_, *selected_bone_local_transform_, *selected_bone_global_transform_);
		update_joint_positions();
	}

	void update_joint_positions()
	{
		// It's fine if no position attribute is selected, this should be called again as soon as one is
		if (!selected_joint_position_)
			return;

		for (const auto& bone : selected_skeleton_->bone_traverser_)
		{
			Vec3 pos = get_basis_position((*selected_bone_global_transform_)[bone]);
			const auto& joints = (*selected_skeleton_->bone_joints_)[bone];
			// Set position for the base joint of the bone
			(*selected_joint_position_)[joints.first] = pos;
			// Also set position for the tip, will be overwritten if the bone isn't a leaf (given the order of the bone traverser)
			(*selected_joint_position_)[joints.second] = pos;
		}

		mesh_provider_->emit_attribute_changed(*selected_skeleton_, selected_joint_position_.get());
	}

	// Transform-type-dependent methods

	template <typename R, typename T>
	TransformT get_current_transform(geometry::KeyframedAnimation<ContainerT, TimeT, geometry::RigidTransformation<R, T>> anim)
	{
		return anim.get_transform(time_, identity_c, geometry::RigidTransformation<R, T>::interpolate);
	}

	TransformT get_current_transform(geometry::KeyframedAnimation<ContainerT, TimeT, geometry::DualQuaternion> anim)
	{
#if DUAL_QUATERNION_USE_SHORTEST_PATH_FOR_INTERPOLATION
		using DQ = geometry::DualQuaternion;
		return anim.get_transform(time_, identity_c,
				[](const DQ& a, const DQ& b, const Scalar& s){ return DQ::lerp(a, b, s, true); });
#else
		return anim.get_transform(time_);
#endif
	}

	template <typename R, typename T>
	Vec3 get_basis_position(geometry::RigidTransformation<R, T> global_transform)
	{
		Vec4 res = global_transform.to_transform_matrix() * Vec4{0, 0, 0, 1};
		return res.head<3>() / res.w();
	}

	Vec3 get_basis_position(geometry::DualQuaternion global_transform)
	{
		return global_transform.transform(Vec3{0, 0, 0});
	}

private:
	TimeT time_ = TimeT{};
	MESH* selected_skeleton_ = nullptr;
	std::shared_ptr<Attribute<AnimationT>> selected_animation_ = nullptr;
	TimeT selected_animation_start_time_;
	TimeT selected_animation_end_time_;
	std::shared_ptr<Attribute<Vec3>> selected_joint_position_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_local_transform_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_global_transform_ = nullptr;
	MeshProvider<MESH>* mesh_provider_ = nullptr;
};

} // namespace ui

} // namespace cgogn

#undef ASC_BIND_INSTANCE_FUNC

#endif // CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_
