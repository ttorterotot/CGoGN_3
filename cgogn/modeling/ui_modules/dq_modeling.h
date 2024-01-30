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

#include <cgogn/geometry/types/vector_traits.h>

#include <boost/synapse/connect.hpp>

namespace cgogn
{

namespace ui
{

using geometry::Mat3;
using geometry::Scalar;
using geometry::Vec3;

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
	void do_something(SURFACE& m, Attribute<Vec3>* vertex_position)
	{
		std::cout << "Placeholder" << std::endl;
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
				if (ImGui::Button("Placeholder"))
				{
					do_something(*selected_surface_, selected_surface_vertex_position_.get());
				}
			}
		}
	}

private:
	MeshProvider<SURFACE>* surface_provider_ = nullptr;

	SURFACE* selected_surface_ = nullptr;
	std::shared_ptr<Attribute<Vec3>> selected_surface_vertex_position_ = nullptr;

	std::vector<std::shared_ptr<boost::synapse::connection>> connections_;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_DQ_MODELING_H_
