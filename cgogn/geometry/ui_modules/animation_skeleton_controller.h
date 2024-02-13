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

	enum class TimePoint
	{
		Start,
		End,
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

	// Static methods

	/// @brief Checks whether an animation attribute has at least one keyframe for each bone.
	/// @param anims the animation attribute to check for
	/// @return whether or not the animation has at least one keyframe for each bone
	static bool is_animation_attribute_complete(const Attribute<AnimationT>& anims)
	{
		return std::find_if(anims.begin(), anims.end(), [](const AnimationT& anim){ return anim.size() == 0; })
				== anims.end();
	}

	/// @brief Checks whether an animation attribute is sorted by time for all bones.
	/// Does not check if there's at least one keyframe for each bone, for that see `is_animation_attribute_complete`.
	/// @param anims the animation attribute to check for
	/// @param warn whether or not to print warnings in the standard output if the animation is found not to be sorted
	/// @return whether or not the animation is sorted
	static bool is_animation_attribute_sorted(const Attribute<AnimationT>& anims)
	{
		return std::find_if_not(anims.begin(), anims.end(), [](const AnimationT& anim){ return anim.is_sorted(); })
				== anims.end();
	}

	/// @brief Computes the first and last keyframe's times across all bones' animations.
	/// If no animation has any keyframe, then `start_time > end_time`.
	/// @param anims the attribute containing the animations
	/// @param start_time the variable to update with the start time
	/// @param end_time the variable to update with the end time
	/// @param all_sorted whether or not all animations can be expected to be sorted
	static void compute_animation_time_extrema(const Attribute<AnimationT>& anims,
			TimeT& start_time, TimeT& end_time, bool all_sorted = false)
	{
		start_time = std::numeric_limits<TimeT>::max();
		end_time = std::numeric_limits<TimeT>::lowest();

		if (all_sorted)
		{
			for (const auto& anim : anims)
			{
				if (anim.size() > 0)
				{
					start_time = std::min(start_time, anim[0].time_);
					end_time = std::max(end_time, anim[anim.size() - 1].time_);
				}
			}
		}
		else
		{
			for (const auto& anim : anims)
			{
				for (const auto& keyframe : anim)
				{
					start_time = std::min(start_time, keyframe.time_);
					end_time = std::max(end_time, keyframe.time_);
				}
			}
		}
	}

	/// @brief Computes local transforms from an animation at a given time.
	/// To compute world transforms from local ones see `compute_world_transforms`.
	/// To compute both local and world transforms with a single method call see `compute_all_transforms`.
	/// @param time the time to compute transforms for from the animations
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to update
	static void compute_local_transforms(
			TimeT time,
			const AnimationSkeleton& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms)
	{
		for (const auto& bone : as.bone_traverser_)
			local_transforms[bone] = get_transform(anims[bone], time);
	}

	/// @brief Computes local transforms from an animation at a given time.
	/// This concatenates calls to `compute_local_transforms` and `compute_world_transforms`.
	/// To compute joint positions from world transforms see `compute_joint_positions`;
	/// to compute them with transforms in a single method call see `compute_everything`.
	/// @param time the time to compute transforms for from the animations
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to update
	/// @param world_transforms the bone world transform attribute to update
	static void compute_all_transforms(
			TimeT time,
			const AnimationSkeleton& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms)
	{
		compute_local_transforms(time, as, anims, local_transforms);
		compute_world_transforms(as, local_transforms, world_transforms);
	}

	/// @brief Updates joint positions from the skeleton and transforms.
	/// Does not send any update signal.
	/// @param as the skeleton the attributes are for
	/// @param world_transforms the bone world transform attribute to use
	/// @param positions the joint position attribute to update
	static void compute_joint_positions(
			const MESH& as,
			const Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions)
	{
		for (const auto& bone : as.bone_traverser_)
		{
			Vec3 pos = get_basis_position(world_transforms[bone]);
			const auto& joints = (*as.bone_joints_)[bone];
			// Set position for the base joint of the bone
			positions[joints.first] = pos;
			// Also set position for the tip, may be overwritten if the bone isn't a leaf (given the order of the bone traverser)
			positions[joints.second] = pos;
		}
	}

	/// @brief Computes transforms and joint positions from an animation at a given time.
	/// @param time the time to compute transforms for from the animations
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to update
	/// @param world_transforms the bone world transform attribute to update
	/// @param positions the joint position attribute to update
	static void compute_everything(
			TimeT time,
			const MESH& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions)
	{
		compute_all_transforms(time, as, anims, local_transforms, world_transforms);
		compute_joint_positions(as, world_transforms, positions);
	}

	/// @brief Computes transforms and joint positions from an animation at a given time.
	/// @param time_point the time to compute transforms for from the animations
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param anims_start_time the variable to update with the start time
	/// @param anims_end_time the variable to update with the end time
	/// @param local_transforms the bone local transform attribute to update
	/// @param world_transforms the bone world transform attribute to update
	/// @param positions the joint position attribute to update
	/// @param assume_all_anims_sorted whether or not all animations can be assumed to be sorted
	static void compute_everything(
			TimePoint time_point,
			const MESH& as,
			const Attribute<AnimationT>& anims,
			TimeT& anims_start_time,
			TimeT& anims_end_time,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions,
			bool assume_all_anims_sorted = false)
	{
		compute_animation_time_extrema(anims, anims_start_time, anims_end_time, assume_all_anims_sorted);

		TimeT time;
		switch (time_point)
		{
		case TimePoint::Start:
			time = anims_start_time;
			break;
		case TimePoint::End:
			time = anims_end_time;
			break;
		default:
			cgogn_assert_not_reached("Missing time point case");
		}

		compute_everything(time, as, anims, local_transforms, world_transforms, positions);
	}

	/// @brief Computes transforms and joint positions from an animation at a given time.
	/// @param time_point the time to compute transforms for from the animations
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to update
	/// @param world_transforms the bone world transform attribute to update
	/// @param positions the joint position attribute to update
	/// @param assume_all_anims_sorted whether or not all animations can be assumed to be sorted
	static void compute_everything(
			TimePoint time_point,
			const MESH& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions,
			bool assume_all_anims_sorted = false)
	{
		TimeT anims_start_time, anims_end_time; // times not cached beyond this call
		compute_everything(time_point, as, anims, anims_start_time, anims_end_time,
				local_transforms, world_transforms, positions,
				assume_all_anims_sorted);
	}

	/// @brief Computes the (ordered) set of keyframes' times across all bones' animations.
	/// @param anims the animation attribute to get keyframes from
	/// @return the set of times of the animation
	[[nodiscard]]
	static std::set<TimeT> get_animation_times(const Attribute<AnimationT>& anims)
	{
		std::set<TimeT> res;

		for (const auto& anim : anims)
			for (const auto& keyframe : anim)
				res.insert(keyframe.time_);

		return res;
	}

	/// @brief Computes the (ordered) set of keyframes' times across all bones' animations.
	/// @param anims the animation attribute to get keyframes from
	/// @param prec the minimum distance from existing times that each new one has to be from to be added
	/// @return the set of times of the animation
	[[nodiscard]]
	static std::set<TimeT> get_animation_times(const Attribute<AnimationT>& anims, TimeT prec)
	{
		std::set<TimeT> res{[&prec](const TimeT& a, const TimeT& b){ return a + prec < b; }};

		for (const auto& anim : anims)
			for (const auto& keyframe : anim)
				res.insert(keyframe.time_);

		return res;
	}

	/// @brief Computes the overarching bounding box across all bones' animations.
	/// If no animation has any keyframe, then for each component `c` of the returned vectors,
	/// `first.c > second.c`, otherwise for each component `first.c <= second.c`.
	/// @param times the times at which to sample the animation (defaults to keyframes' times)
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to use as a buffer
	/// @param world_transforms the bone world transform attribute to use as a buffer
	/// @param positions the joint position attribute to use as a buffer
	/// @return a pair of vectors representing the corner points of the bounding box
	template <template <typename> typename ContainerU>
	[[nodiscard]]
	static std::pair<Vec3, Vec3> compute_animation_bb(
			const ContainerU<TimeT>& times,
			const MESH& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions)
	{
		constexpr const auto SCALAR_MIN = std::numeric_limits<Vec3::Scalar>::lowest();
		constexpr const auto SCALAR_MAX = std::numeric_limits<Vec3::Scalar>::max();
		Vec3 bb_min = {SCALAR_MAX, SCALAR_MAX, SCALAR_MAX};
		Vec3 bb_max = {SCALAR_MIN, SCALAR_MIN, SCALAR_MIN};

		for (const auto& time : times)
		{
			compute_everything(time, as, anims, local_transforms, world_transforms, positions);

			for (const Vec3& position : positions)
			{
				bb_min = bb_min.cwiseMin(position);
				bb_max = bb_max.cwiseMax(position);
			}
		}

		return std::make_pair(bb_min, bb_max);
	}

	/// @brief Computes the overarching bounding box across all bones' animations.
	/// If no animation has any keyframe, then for each component `c` of the returned vectors,
	/// `first.c > second.c`, otherwise for each component `first.c <= second.c`.
	/// For animations with nonlinear transformations such as rotations,
	/// interpolated positions may fall outside the bounding box of positions at keyframes,
	/// thus it may be recommended to use the overload of this method with custom times.
	/// If the animations have many unequal but close keyframes, look into using that overload
	/// with `get_animation_times(..., prec)`.
	/// @param as the skeleton the attributes are for
	/// @param anims the animation attribute to get transforms from
	/// @param local_transforms the bone local transform attribute to use as a buffer
	/// @param world_transforms the bone world transform attribute to use as a buffer
	/// @param positions the joint position attribute to use as a buffer
	/// @return a pair of vectors representing the corner points of the bounding box
	[[nodiscard]]
	static std::pair<Vec3, Vec3> compute_animation_bb(
			const MESH& as,
			const Attribute<AnimationT>& anims,
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions,
			bool assume_all_anims_sorted = false)
	{
		return compute_animation_bb(get_animation_times(anims), as, anims,
				local_transforms, world_transforms, positions);
	}

	// Transform-type-dependent static methods

	template <typename R, typename T>
	static TransformT get_transform(const geometry::KeyframedAnimation<ContainerT, TimeT,
					geometry::RigidTransformation<R, T>>& anim,
			TimeT time)
	{
		return anim.get_transform(time, identity_c, geometry::RigidTransformation<R, T>::interpolate);
	}

	static TransformT get_transform(const geometry::KeyframedAnimation<ContainerT, TimeT,
					geometry::DualQuaternion>& anim,
			TimeT time, bool shortest_path = true)
	{
		if (!shortest_path)
			return anim.get_transform(time);

		using DQ = geometry::DualQuaternion;
		return anim.get_transform(time, identity_c,
				[](const DQ& a, const DQ& b, const Scalar& s){ return DQ::lerp(a, b, s, true); });
	}

	template <typename R, typename T>
	static Vec3 get_basis_position(const geometry::RigidTransformation<R, T>& world_transform)
	{
		Vec4 res = world_transform.to_transform_matrix() * Vec4{0, 0, 0, 1};
		return res.head<3>() / res.w();
	}

	static Vec3 get_basis_position(const geometry::DualQuaternion& world_transform)
	{
		return world_transform.transform(Vec3{0, 0, 0});
	}

	// Instance methods

	/// @brief Changes the linked bone animation attribute.
	/// Does not affect the embedding, see `set_time` and `update_embedding`.
	/// @param attribute the new attribute to use as animations
	/// @param check_params what to check about animation validity (size and sorting)
	void set_animation(const std::shared_ptr<Attribute<AnimationT>>& attribute, CheckParams check_params = CheckParams::CheckAndWarnInvalid)
	{
		selected_animation_ = attribute;

		if (!selected_animation_)
			return; // animation unset

		// Check for empty animations
		cgogn_message_assert(check_params == CheckParams::AssumeValid || is_animation_attribute_complete(*selected_animation_),
				"[AnimationSkeletonController::set_animation] Found empty animation");

		// Check if all bones' animations are sorted
		bool all_sorted = check_params == CheckParams::AssumeValid || is_animation_attribute_sorted(*selected_animation_);

		if (!all_sorted && check_params == CheckParams::CheckAndWarnInvalid)
			std::cout << "[AnimationSkeletonController::set_animation] Found unsorted animation, "
					"skeleton may be animated improperly" << std::endl;

		compute_animation_time_extrema(*selected_animation_,
				selected_animation_start_time_, selected_animation_end_time_, all_sorted);
	}

	/// @brief Changes the linked joint position attribute, and updates it if it's not null and a skeleton is selected.
	/// Assumes that if a skeleton is selected, its world transform attribute also is.
	/// @param attribute the new attribute to use as positions
	void set_joint_position(const std::shared_ptr<Attribute<Vec3>>& attribute)
	{
		selected_joint_position_ = attribute;

		if (selected_skeleton_ && selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	/// @brief Changes the current time of the animation.
	/// Updates positions accordingly if a skeleton and animation are selected.
	/// @param time the new time to set the animation to
	void set_time(TimeT time)
	{
		time_ = time;

		if (selected_skeleton_ && selected_animation_)
			update_embedding();
	}

	/// @brief Changes the current time of the animation, and updates positions accordingly if a skeleton is selected.
	/// @param time_point the new time to set the animation to
	void set_time(TimePoint time_point)
	{
		if (!selected_animation_)
			return;

		switch (time_point)
		{
		case TimePoint::Start:
			set_time(selected_animation_start_time_);
			break;
		case TimePoint::End:
			set_time(selected_animation_end_time_);
			break;
		default:
			cgogn_assert_not_reached("Missing time point case");
		}
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
			selected_bone_local_transform_ = cgogn::get_or_add_attribute<TransformT, Bone>(m, LOCAL_TRANSFORM_ATTRIBUTE_NAME);
			selected_bone_world_transform_ = cgogn::get_or_add_attribute<TransformT, Bone>(m, WORLD_TRANSFORM_ATTRIBUTE_NAME);
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
	// Updates transforms and joint positions.
	// Assumes a skeleton and animation are selected as well as the corresponding transform attributes.
	void update_embedding()
	{
		cgogn_assert(selected_skeleton_ && selected_animation_
				&& selected_bone_world_transform_ && selected_bone_world_transform_);

		compute_all_transforms(time_, *selected_skeleton_, *selected_animation_,
				*selected_bone_local_transform_, *selected_bone_world_transform_);

		// It's fine if no position attribute is selected, this should be called again as soon as one is
		if (selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	// Updates joint positions from the skeleton and transforms,
	// sending an update signal through the mesh provider.
	static void update_joint_positions_and_signal(
			MeshProvider<MESH>* mesh_provider,
			const MESH& as,
			const Attribute<TransformT>& world_transforms,
			Attribute<Vec3>* positions)
	{
		compute_joint_positions(as, world_transforms, *positions);

		if (mesh_provider)
			mesh_provider->emit_attribute_changed(as, positions);
	}

public:
	static constexpr const char* LOCAL_TRANSFORM_ATTRIBUTE_NAME = "local_transform";
	static constexpr const char* WORLD_TRANSFORM_ATTRIBUTE_NAME = "world_transform";

private:
	TimeT time_ = TimeT{};
	MESH* selected_skeleton_ = nullptr;
	std::shared_ptr<Attribute<AnimationT>> selected_animation_ = nullptr;
	TimeT selected_animation_start_time_;
	TimeT selected_animation_end_time_;
	std::shared_ptr<Attribute<Vec3>> selected_joint_position_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_local_transform_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_world_transform_ = nullptr;
	MeshProvider<MESH>* mesh_provider_ = nullptr;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_
