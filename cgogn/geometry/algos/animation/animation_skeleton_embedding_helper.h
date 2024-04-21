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

#ifndef CGOGN_GEOMETRY_ANIMATION_SKELETON_EMBEDDING_H_
#define CGOGN_GEOMETRY_ANIMATION_SKELETON_EMBEDDING_H_

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/animation/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>

namespace cgogn
{

struct MapBase;

namespace geometry
{

template <template <typename> typename ContainerT, typename TimeT, typename TransformT>
class AnimationSkeletonEmbeddingHelper
{
public:
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

	AnimationSkeletonEmbeddingHelper() = delete;

	// Transforms and positions

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
		for (auto i = static_cast<std::make_signed_t<size_t>>(as.nb_bones()) - 1; i >= 0; --i)
		{
			const auto& bone = as.bone_traverser_[i];
			Vec3 pos = get_basis_position(world_transforms[bone]);
			const auto& joints = (*as.bone_joints_)[bone];
			// Set position for the base joint of the bone
			positions[joints.first] = pos;
			// Set position for the tip joint of the bone, will be overwritten if the bone isn't a root
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
			Attribute<TransformT>& local_transforms,
			Attribute<TransformT>& world_transforms,
			Attribute<Vec3>& positions,
			bool assume_all_anims_sorted = false)
	{
		auto times = AnimationT::compute_keyframe_time_extrema(anims, assume_all_anims_sorted);

		TimeT time{};
		if (times)
			switch (time_point)
			{
			case TimePoint::Start:
				time = times->first;
				break;
			case TimePoint::End:
				time = times->second;
				break;
			default:
				cgogn_assert_not_reached("Missing time point case");
			}

		compute_everything(time, as, anims, local_transforms, world_transforms, positions);
	}

	// Bounding box

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
	template <typename ContainerU>
	[[nodiscard]]
	static std::pair<Vec3, Vec3> compute_animation_bb(
			const ContainerU& times,
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
		return compute_animation_bb(AnimationT::get_unique_keyframe_times(anims), as, anims,
				local_transforms, world_transforms, positions);
	}

	// Color

	/// @brief Generates colors loosely based on the skeleton's topology
	/// @param sk the skeleton the attribute is for
	/// @param bone_color the bone color attribute to write to
	static void generate_bone_colors(const MESH& sk, Attribute<Vec3>& bone_color)
	{
		constexpr const Vec3::Scalar decay_factor = 0.75;
		for (const auto& bone : sk.bone_traverser_)
			bone_color[bone] = is_root(sk, bone) ? Vec3::Ones() : Vec3{decay_factor * bone_color[(*sk.bone_parent_)[bone]]};
	}

	// Transform-type-dependent methods

	template <typename R, typename T>
	static TransformT get_transform(const geometry::KeyframedAnimation<ContainerT, TimeT,
					geometry::RigidTransformation<R, T>>& anim,
			TimeT time)
	{
		return anim.get_value(time, identity_c, geometry::RigidTransformation<R, T>::interpolate);
	}

	static TransformT get_transform(const geometry::KeyframedAnimation<ContainerT, TimeT,
					geometry::DualQuaternion>& anim,
			TimeT time, bool shortest_path = true)
	{
		if (!shortest_path)
			return anim.get_value(time).normalized();

		using DQ = geometry::DualQuaternion;
		return anim.get_value(time, identity_c,
				[](const DQ& a, const DQ& b, const Scalar& s){ return DQ::lerp(a, b, s, true); })
						.normalized();
	}

	template <typename R, typename T>
	static inline Vec3 get_basis_position(const geometry::RigidTransformation<R, T>& world_transform)
	{
		using RT = geometry::RigidTransformation<R, T>;
		return Eigen::Translation<typename RT::Scalar, RT::Dim>{world_transform.translation()}.translation();
	}

	static inline Vec3 get_basis_position(const geometry::DualQuaternion& world_transform)
	{
		return world_transform.translation();
	}
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ANIMATION_SKELETON_EMBEDDING_H_
