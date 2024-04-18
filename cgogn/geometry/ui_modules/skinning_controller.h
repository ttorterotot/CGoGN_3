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

#ifndef CGOGN_MODULE_SKINNING_CONTROLLER_H_
#define CGOGN_MODULE_SKINNING_CONTROLLER_H_

#include <boost/core/demangle.hpp>

#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/geometry/algos/animation/skinning_helper.h>

namespace cgogn
{

namespace ui
{

using geometry::Vec3;
using geometry::Vec4;
using geometry::Vec4i;
using geometry::Scalar;

template <typename Mesh, typename TransformT>
class SkinningController : public Module
{
public:
	using Skinning = geometry::SkinningHelper<TransformT>;

protected:
	using Skeleton = AnimationSkeleton;

	template <typename T>
	using AttributeSk = AnimationSkeleton::Attribute<T>;

	using Bone = AnimationSkeleton::Bone;

	template <typename T>
	using AttributeSf = typename cgogn::mesh_traits<Mesh>::template Attribute<T>;

	using Vertex = typename cgogn::mesh_traits<Mesh>::Vertex;

public:
	SkinningController(const App& app,
			const std::string& mesh_bind_position_attribute_unique_name = "bind_vertex_position",
			const std::string& skeleton_bind_pose_attribute_unique_name = "bind_bone_inv_world_transform_"
					+ get_demangled_transform_name()) :
		Module(app, "SkinningController (" + std::string{mesh_traits<Skeleton>::name}
				+ ", " + get_demangled_mesh_name() + ", " + get_demangled_transform_name() + ")"),
		bind_vertex_position_attribute_name_(mesh_bind_position_attribute_unique_name),
		bind_inv_world_transform_attribute_name_(skeleton_bind_pose_attribute_unique_name)
	{
	}
	~SkinningController()
	{
	}

	/// @brief Changes the linked vertex position attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as positions
	void set_vertex_position(const std::shared_ptr<AttributeSf<Vec3>>& attribute)
	{
		selected_vertex_position_ = attribute;

		if (!selected_mesh_)
			return;

		selected_bind_vertex_position_
				= cgogn::get_or_add_attribute<Vec3, Vertex>(*selected_mesh_, bind_vertex_position_attribute_name_);

		if (attribute)
			selected_bind_vertex_position_->copy(*attribute);

		update_embedding();
	}

	/// @brief Changes the linked vertex weight index attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight indices
	void set_vertex_weight_index(const std::shared_ptr<AttributeSf<Vec4i>>& attribute)
	{
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_ = attribute;
		update_embedding();
	}

	/// @brief Changes the linked vertex weight value attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as weight values
	void set_vertex_weight_value(const std::shared_ptr<AttributeSf<Vec4>>& attribute)
	{
		selected_vertex_weight_value_ = attribute;
		update_embedding();
	}

	/// @brief Changes the linked vertex weight value attribute, and updates the mesh if possible.
	/// @param attribute the new attribute to use as transforms
	void set_bone_world_transform(const std::shared_ptr<AttributeSk<TransformT>>& attribute)
	{
		selected_bone_world_transform_ = attribute;

		if (!selected_skeleton_)
			return;

		selected_bind_bone_inv_world_transform_
				= cgogn::get_or_add_attribute<TransformT, Bone>(*selected_skeleton_, bind_inv_world_transform_attribute_name_);

		if (attribute)
		{
			selected_bind_bone_inv_world_transform_->copy(*attribute);

			for (auto& t : *selected_bind_bone_inv_world_transform_)
				t = t.inverse();
		}

		update_embedding();
	}

	/// @brief Changes the linked mesh, and resets attribute selection for it.
	/// @param sf the new mesh to link to
	void set_mesh(Mesh* sf)
	{
		selected_mesh_ = sf;
		selected_vertex_weight_index_valid_ = true;
		selected_vertex_weight_index_.reset();
		selected_vertex_weight_value_.reset();
		set_vertex_position(cgogn::get_attribute<Vec3, Vertex>(*sf, "position")); // nullptr (equiv. to reset) if not found
	}

	/// @brief Changes the linked skeleton, and resets attribute selection for it.
	/// @param sk the new skeleton to link to
	void set_skeleton(Skeleton* sk)
	{
		selected_skeleton_= sk;
		set_bone_world_transform(nullptr);

		skeleton_connection_ = !sk ? nullptr
				: boost::synapse::connect<typename MeshProvider<Mesh>::template attribute_changed_t<TransformT>>(
						sk, [&](AttributeSk<TransformT>* attribute)
				{
					if (selected_bone_world_transform_.get() == attribute)
						update_embedding();
				});
	}

	/// @return the attribute name for the bind position
	[[nodiscard]]
	const std::string& mesh_bind_position_attribute_name() const
	{
		return bind_vertex_position_attribute_name_;
	}

	/// @return the attribute name for the bind pose
	[[nodiscard]]
	const std::string& skeleton_bind_pose_attribute_name() const
	{
		return bind_inv_world_transform_attribute_name_;
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<ui::MeshProvider<Mesh>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<Mesh>::name} + ")"));
		skeleton_provider_ = static_cast<ui::MeshProvider<Skeleton>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<Skeleton>::name} + ")"));
	}

	void left_panel() override
	{
		imgui_mesh_selector(mesh_provider_, selected_mesh_, "Mesh", [&](Mesh& sf) { set_mesh(&sf); });

		if (selected_mesh_)
		{
			imgui_combo_attribute<Vertex, Vec3>(*selected_mesh_, selected_vertex_position_,
					"Position", [&](const std::shared_ptr<AttributeSf<Vec3>>& attribute){ set_vertex_position(attribute); });
			imgui_combo_attribute<Vertex, Vec4i>(*selected_mesh_, selected_vertex_weight_index_,
					"Weight indices", [&](const std::shared_ptr<AttributeSf<Vec4i>>& attribute){ set_vertex_weight_index(attribute); });
			imgui_combo_attribute<Vertex, Vec4>(*selected_mesh_, selected_vertex_weight_value_,
					"Weight values", [&](const std::shared_ptr<AttributeSf<Vec4>>& attribute){ set_vertex_weight_value(attribute); });

			if (!selected_vertex_weight_index_valid_)
			{
				ImGui::Text("Invalid weight indices found");
				ImGui::Text("Is the right skeleton selected?");
			}

			if constexpr (USE_LBS_)
				if (ImGui::Checkbox("Normalize weights", &normalize_weights_))
					update_embedding();
		}

		imgui_mesh_selector(skeleton_provider_, selected_skeleton_, "Skeleton", [&](Skeleton& sk) { set_skeleton(&sk); });

		if (selected_skeleton_)
		{
			imgui_combo_attribute<Bone, TransformT>(*selected_skeleton_, selected_bone_world_transform_,
					"World transform", [&](const std::shared_ptr<AttributeSk<TransformT>>& attribute){ set_bone_world_transform(attribute); });
		}
	}

private:
	static std::string get_demangled_mesh_name()
	{
		return boost::core::demangle(typeid(Mesh).name());
	}

	static std::string get_demangled_transform_name()
	{
		return boost::core::demangle(typeid(TransformT).name());
	}

	void update_embedding()
	{
		if (!selected_vertex_position_ || !selected_vertex_weight_index_ || !selected_vertex_weight_value_
				|| !selected_bone_world_transform_)
			return;

		if constexpr (USE_LBS_)
			selected_vertex_weight_index_valid_ = Skinning::compute_vertex_positions_LBS(
					*selected_mesh_, *selected_skeleton_,
					*selected_bind_bone_inv_world_transform_, *selected_bone_world_transform_,
					*selected_vertex_weight_index_, *selected_vertex_weight_value_,
					*selected_bind_vertex_position_, *selected_vertex_position_,
					normalize_weights_);
		else
			selected_vertex_weight_index_valid_ = Skinning::compute_vertex_positions_TBS(
					*selected_mesh_, *selected_skeleton_,
					*selected_bind_bone_inv_world_transform_, *selected_bone_world_transform_,
					*selected_vertex_weight_index_, *selected_vertex_weight_value_,
					*selected_bind_vertex_position_, *selected_vertex_position_);

		if (mesh_provider_)
			mesh_provider_->emit_attribute_changed(*selected_mesh_, selected_vertex_position_.get());
	}

private:
	static constexpr const bool USE_LBS_ = !std::is_same_v<TransformT, geometry::DualQuaternion>;
	Mesh* selected_mesh_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_vertex_position_ = nullptr;
	std::shared_ptr<AttributeSf<Vec3>> selected_bind_vertex_position_ = nullptr;
	bool selected_vertex_weight_index_valid_ = true;
	std::shared_ptr<AttributeSf<Vec4i>> selected_vertex_weight_index_ = nullptr;
	std::shared_ptr<AttributeSf<Vec4>> selected_vertex_weight_value_ = nullptr;
	bool normalize_weights_ = true;
	std::string bind_vertex_position_attribute_name_;
	MeshProvider<Mesh>* mesh_provider_ = nullptr;
	Skeleton* selected_skeleton_ = nullptr;
	std::shared_ptr<AttributeSk<TransformT>> selected_bone_world_transform_ = nullptr;
	std::shared_ptr<AttributeSk<TransformT>> selected_bind_bone_inv_world_transform_ = nullptr;
	std::string bind_inv_world_transform_attribute_name_ = nullptr;
	std::shared_ptr<boost::synapse::connection> skeleton_connection_ = nullptr;
	MeshProvider<Skeleton>* skeleton_provider_ = nullptr;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_SKINNING_CONTROLLER_H_
