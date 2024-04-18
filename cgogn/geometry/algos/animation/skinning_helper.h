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

#ifndef CGOGN_SKINNING_HELPER_H_
#define CGOGN_SKINNING_HELPER_H_

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>

namespace cgogn
{

namespace geometry
{

template <typename Surface, typename TransformT>
class SkinningHelper
{
private:
	using AffineTransform = Eigen::Transform<Scalar, 3, Eigen::Affine>;

	using Skeleton = AnimationSkeleton;

	template <typename T>
	using AttributeSf = typename mesh_traits<Surface>::template Attribute<T>;

	using Vertex = typename mesh_traits<Surface>::Vertex;

	template <typename T>
	using AttributeSk = AnimationSkeleton::Attribute<T>;

	using Bone = AnimationSkeleton::Bone;

	// Transform type to pre-computed transform type

	template <typename T>
	struct PrecomputedTransformTmpl
	{
		using Type = Mat4;

		static Type from(const TransformT& world_transform)
		{
			if constexpr (std::is_same_v<TransformT, Mat4>)
				return world_transform;
			else
				return world_transform.to_transform_matrix();
		}
	};

	template <typename R, typename T>
	struct PrecomputedTransformTmpl<RigidTransformation<R, T>>
	{
		using Type = AffineTransform;

		static Type from(const TransformT& world_transform)
		{
			return world_transform.to_transform();
		}
	};

	using PrecomputedTransform = PrecomputedTransformTmpl<TransformT>;

public:

	SkinningHelper() = delete;

	/// @brief Updates vertex positions from the skeleton and transforms.
	/// Positions are computed for all affecting transforms,
	/// then a weighted average of them is made.
	/// To interpolate transforms instead, see `compute_vertex_positions_TBS`.
	/// Does not send any update signal.
	/// @param m the mesh the vertex attributes are for
	/// @param as the skeleton the bone attributes are for
	/// @param bind_inv_world_transforms the inverse bind world transform attribute to use
	/// @param world_transforms the current world transform attribute to use
	/// @param weight_indices the affecting bone index attribute to use
	/// @param weight_values the affecing bone weight attribute to use
	/// @param bind_positions the bind vertex position attribute to use
	/// @param positions the vertex position attribute to update
	/// @param normalize_weights whether or not to normalize weights if they're not already
	/// @return whether or not all set (!= -1) bone indices could refer to an actual bone in the skeleton
	template <typename MESH>
	static bool compute_vertex_positions_LBS(
			const MESH& m,
			const AnimationSkeleton& as,
			const AttributeSk<TransformT>& bind_inv_world_transforms,
			const AttributeSk<TransformT>& world_transforms,
			const AttributeSf<Vec4i>& weight_indices,
			const AttributeSf<Vec4>& weight_values,
			const AttributeSf<Vec3>& bind_positions,
			AttributeSf<Vec3>& positions,
			bool normalize_weights = false)
	{
		std::vector<TransformT> raw_offsets = get_offsets(as, bind_inv_world_transforms, world_transforms);
		std::vector<typename PrecomputedTransform::Type> offsets;
		bool res = true;

		// Pre-compute offset transformation matrices
		offsets.reserve(raw_offsets.size());
		std::transform(raw_offsets.cbegin(), raw_offsets.cend(), std::back_inserter(offsets),
				[](const TransformT& world_transform) { return PrecomputedTransform::from(world_transform); });

		parallel_foreach_cell(m, [&](typename mesh_traits<MESH>::Vertex v)
		{
			const auto i = index_of(m, v);
			Vec3 p = Vec3::Zero();
			Vec4::Scalar w = 0.0;

			for (int j = 0; j < 4; ++j)
			{
				auto wi = weight_indices[i][j];
				if (wi < 0)
					continue;
				if (wi >= offsets.size())
				{
					res = false;
					continue;
				}

				auto wv = weight_values[i][j];
				p += wv * transform_point(bind_positions[i], offsets[wi]);
				w += wv;
			}

			if (normalize_weights)
				p /= w;

			positions[i] = p;

			return true;
		});

		return res;
	}

	/// @brief Updates vertex positions from the skeleton and transforms.
	/// Transforms are averaged, then the position is computed from the result.
	/// To interpolate positions instead, see `compute_vertex_positions_LBS`.
	/// Does not send any update signal.
	/// @param m the mesh the vertex attributes are for
	/// @param as the skeleton the bone attributes are for
	/// @param bind_inv_world_transforms the inverse bind world transform attribute to use
	/// @param world_transforms the current world transform attribute to use
	/// @param weight_indices the affecting bone index attribute to use
	/// @param weight_values the affecing bone weight attribute to use
	/// @param bind_positions the bind vertex position attribute to use
	/// @param positions the vertex position attribute to update
	/// @return whether or not all set (!= -1) bone indices could refer to an actual bone in the skeleton
	template <typename MESH>
	static bool compute_vertex_positions_TBS(
			const MESH& m,
			const AnimationSkeleton& as,
			const AttributeSk<TransformT>& bind_inv_world_transforms,
			const AttributeSk<TransformT>& world_transforms,
			const AttributeSf<Vec4i>& weight_indices,
			const AttributeSf<Vec4>& weight_values,
			const AttributeSf<Vec3>& bind_positions,
			AttributeSf<Vec3>& positions)
	{
		static_assert(std::is_same_v<TransformT, DualQuaternion>, "transform type unsupported, use LBS");

		std::vector<TransformT> offsets = get_offsets(as, bind_inv_world_transforms, world_transforms);
		bool res = true;

		parallel_foreach_cell(m, [&](typename mesh_traits<MESH>::Vertex v)
		{
			const auto i = index_of(m, v);
			DualQuaternion t = DualQuaternion::zero();

			for (int j = 0; j < 4; ++j)
			{
				auto wi = weight_indices[i][j];
				if (wi < 0)
					continue;

				if (wi < offsets.size())
					t += weight_values[i][j] * offsets[wi];
				else
					res = false;
			}

			if (t.squaredMagnitude() > 0.0)
				t.normalize();

			positions[i] = t.transform(bind_positions[i]);

			return true;
		});

		return res;
	}

private:

	// Computes the transform offset for the pose.
	// Kept private because the use of std::vector is an implementation choice.
	static std::vector<TransformT> get_offsets(const AnimationSkeleton& as,
			const AttributeSk<TransformT>& bind_inv_world_transforms,
			const AttributeSk<TransformT>& world_transforms)
	{
		// By using a vector that might be larger we avoid using an attribute,
		// or setting offset[i] which requires calling std::find on the bone traverser for retrieval
		// The return value of `maximum_index` is always higher than the highest actual index
		// (see `AttributeContainerGen::new_index`)
		std::vector<TransformT> offsets(world_transforms.maximum_index());

		for (const auto& bone : as.bone_traverser_)
			offsets[bone] = world_transforms[bone] * bind_inv_world_transforms[bone];

		return offsets;
	}

	// Pre-computed-transform-type dependent methods

	static inline Vec3 transform_point(const Vec3& v, const AffineTransform& world_transform)
	{
		return world_transform * v;
	}

	static inline Vec3 transform_point(const Vec3& v, const Mat4& world_transform)
	{
		Vec4 res = world_transform * v.homogeneous().eval();
		return res.head<3>() / res.w();
	}
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_SKINNING_HELPER_H_
