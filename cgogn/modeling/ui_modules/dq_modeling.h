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

#ifndef CGOGN_MODULE_DQ_MODELING_H_
#define CGOGN_MODULE_DQ_MODELING_H_

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/geometry/types/dual_quaternion.h>
#include <cgogn/geometry/types/vector_traits.h>

#include <boost/synapse/connect.hpp>

namespace cgogn
{

namespace ui
{

using geometry::Mat3;
using geometry::Scalar;
using geometry::Vec3;
using geometry::Quaternion;
using geometry::DualQuaternion;

template <typename SURFACE>
class DQModeling : public Module
{
	template <typename T>
	using Attribute = typename mesh_traits<SURFACE>::template Attribute<T>;

	using Vertex = typename mesh_traits<SURFACE>::Vertex;
	using Edge = typename mesh_traits<SURFACE>::Edge;
	using Face = typename mesh_traits<SURFACE>::Face;

public:
	DQModeling(const App& app)
		: Module(app, "DQModeling")
	{
	}

	~DQModeling()
	{
	}

private:
	void init_surface_mesh(SURFACE* s)
	{
	}

public:
	void interpolate_pose(MeshProvider<SURFACE>& mesh_provider,
			const SURFACE& selected_mesh, Attribute<Vec3>* vertex_position,
			float t, DualQuaternion q)
	{
		Attribute<Vec3>& positions = *vertex_position;
		q = DualQuaternion::lerp(DualQuaternion::identity(), q, t);
		DualQuaternion qi{q};

		for (int i = 0; i < positions.size(); i += 2)
		{
			qi.normalize();
			positions[i] = qi.transform({0, 0, 0}); // helix (cumulative transforms)
			positions[i + 1] = qi.transform({Scalar(i), 1, 0}); // initial-position-dependent spiral
			qi *= q;
		}

		mesh_provider.emit_attribute_changed(selected_mesh, vertex_position);
	}

protected:
	void init() override
	{
		surface_provider_ = static_cast<ui::MeshProvider<SURFACE>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<SURFACE>::name} + ")"));

		surface_provider_->foreach_mesh([this](SURFACE& m, const std::string&) { init_surface_mesh(&m); });
		connections_.push_back(boost::synapse::connect<typename MeshProvider<SURFACE>::mesh_added>(
			surface_provider_, this, &DQModeling<SURFACE>::init_surface_mesh));
	}

	void left_panel() override
	{
		imgui_mesh_selector(surface_provider_, selected_surface_, "Surface", [&](SURFACE& m) {
			selected_surface_ = &m;
			selected_surface_vertex_position_.reset();
		});

		if (selected_surface_)
		{
			imgui_combo_attribute<Vertex, Vec3>(*selected_surface_, selected_surface_vertex_position_,
													   "Position",
													   [&](const std::shared_ptr<Attribute<Vec3>>& attribute) {
														   selected_surface_vertex_position_ = attribute;
													   });

			if (selected_surface_vertex_position_)
			{
				if (ImGui::SliderFloat("Pose", &pose_, 0.f, 1.f))
				{
					interpolate_pose(*surface_provider_,
							*selected_surface_, selected_surface_vertex_position_.get(), pose_, pose_q_);
				}
			}
		}
	}

private:
	MeshProvider<SURFACE>* surface_provider_ = nullptr;

	SURFACE* selected_surface_ = nullptr;
	std::shared_ptr<Attribute<Vec3>> selected_surface_vertex_position_ = nullptr;

	std::vector<std::shared_ptr<boost::synapse::connection>> connections_;

	float pose_ = 0.f;
	DualQuaternion pose_q_ = DualQuaternion::from_rt(
			Quaternion{Eigen::AngleAxisd(0.25 * M_PI, Vec3::UnitZ())}, {0, 0.25, 1});
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_DQ_MODELING_H_
