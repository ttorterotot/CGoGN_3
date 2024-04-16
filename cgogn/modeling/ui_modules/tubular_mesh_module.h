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

#ifndef CGOGN_MODULE_TUBULAR_MESH_H_
#define CGOGN_MODULE_TUBULAR_MESH_H_

#include <cgogn/modeling/ui_modules/volume_surface_fitting.h>

#include <cgogn/geometry/types/vector_traits.h>

#include <cgogn/core/functions/traversals/vertex.h>

#include <cgogn/geometry/algos/length.h>

#include <cgogn/modeling/algos/graph_resampling.h>
#include <cgogn/modeling/algos/graph_to_hex.h>
#include <cgogn/modeling/algos/incidenceGraph_to_hex.h>
#include <cgogn/modeling/algos/subdivision/basic.h>
#include <cgogn/modeling/algos/volume_utils.h>

namespace cgogn
{

namespace ui
{

using geometry::Scalar;
using geometry::Vec3;

template <typename GRAPH, typename SURFACE, typename VOLUME>
class TubularMesh : public VolumeSurfaceFitting<SURFACE, VOLUME, false>
{
	using Base = VolumeSurfaceFitting<SURFACE, VOLUME, false>;

	// Protected members need to be specified due to two-phase lookup

	using Base::set_volume_caches_dirty;

	using Base::app_;
	using Base::surface_provider_;
	using Base::volume_provider_;
	using Base::surface_;
	using Base::surface_vertex_position_;
	using Base::contact_surface_;
	using Base::volume_;
	using Base::volume_vertex_position_;
	using Base::volume_skin_;
	using Base::volume_skin_vertex_position_;
	using Base::volume_skin_vertex_volume_vertex_;

	template <typename T>
	using GraphAttribute = typename mesh_traits<GRAPH>::template Attribute<T>;

	using GraphVertex = typename mesh_traits<GRAPH>::Vertex;
	using GraphEdge = typename mesh_traits<GRAPH>::Edge;

	using SurfaceVertex = typename mesh_traits<SURFACE>::Vertex;
	using SurfaceFace = typename mesh_traits<SURFACE>::Face;

	using VolumeVertex = typename mesh_traits<VOLUME>::Vertex;
	using VolumeEdge = typename mesh_traits<VOLUME>::Edge;
	using VolumeFace = typename mesh_traits<VOLUME>::Face;

public:
	using Base::closest_surface_point;
	using Base::closest_surface_face_and_point;
	using Base::set_current_volume;
	using Base::refresh_volume_skin;
	using Base::intersect_bvh;

public:
	TubularMesh(const App& app) : Base(app, "TubularMesh")
	{
	}

protected:
	void init() override
	{
		Base::init();
		graph_provider_ = static_cast<ui::MeshProvider<GRAPH>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<GRAPH>::name} + ")"));
	}

public:
	void init_graph_radius_from_edge_length()
	{
		Scalar l = geometry::mean_edge_length(*graph_, graph_vertex_position_.get());
		graph_vertex_radius_->fill(l); // / 2.0);
		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_radius_.get());
	}

	void extend_graph_extremities()
	{
		using SelectedFace = std::tuple<SurfaceFace, Vec3, Scalar>;
		CellCache<GRAPH> cache(*graph_);
		cache.template build<GraphVertex>();
		foreach_cell(cache, [&](GraphVertex v) -> bool {
			if (degree(*graph_, v) == 1)
			{
				std::vector<GraphVertex> av = adjacent_vertices_through_edge(*graph_, v);
				const Vec3& p = value<Vec3>(*graph_, graph_vertex_position_, v);
				const Vec3& q = value<Vec3>(*graph_, graph_vertex_position_, av[0]);
				Vec3 dir = p - q;

				auto h = intersect_bvh({p, dir, 0, acc::inf});
				if (h.hit)
				{
					GraphVertex nv = add_vertex(*graph_);
					connect_vertices(*graph_, v, nv);
					value<Vec3>(*graph_, graph_vertex_position_, nv) = p + 0.6 * (h.pos - p);
				}
			}
			return true;
		});

		graph_provider_->emit_connectivity_changed(*graph_);
		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_position_.get());
	}

	void recenter_graph_from_surface()
	{
		parallel_foreach_cell(*graph_, [&](GraphVertex v) -> bool {
			Vec3& p = value<Vec3>(*graph_, graph_vertex_position_, v);
			Vec3 cp = closest_surface_point(p);
			Scalar prev_r;
			Scalar r = (cp - p).norm();
			// Vec3 prev_displ;
			Vec3 displ = 0.01 * (p - cp);
			p += displ;
			do
			{
				prev_r = r;
				// prev_displ = displ;
				cp = closest_surface_point(p);
				r = (cp - p).norm();
				displ = 0.01 * (p - cp);
				p += displ;
			} while (prev_r > r);
			// } while (prev_displ.dot(displ) > 0);

			cp = closest_surface_point(p);
			value<Scalar>(*graph_, graph_vertex_radius_, v) = (cp - p).norm();

			return true;
		});

		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_position_.get());
		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_radius_.get());
		// cgogn::io::export_CGR(*graph_, graph_vertex_position_.get(), graph_vertex_radius_.get(), "export.cgr");
	}

	void init_graph_radius_from_surface()
	{
		parallel_foreach_cell(*graph_, [&](GraphVertex v) -> bool {
			const Vec3& p = value<Vec3>(*graph_, graph_vertex_position_, v);
			Vec3 cp = closest_surface_point(p);
			value<Scalar>(*graph_, graph_vertex_radius_, v) = (cp - p).norm() * 0.5;
			return true;
		});
		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_radius_.get());
		// cgogn::io::export_CGR(*graph_, graph_vertex_position_.get(), graph_vertex_radius_.get(), "export.cgr");
	}

	GRAPH* resample_graph(Scalar density)
	{
		if constexpr (std::is_same_v<GRAPH, cgogn::Graph>)
		{
			static uint32 count = 0;
			GRAPH* resampled_graph = graph_provider_->add_mesh("resampled_" + std::to_string(count++));
			auto resampled_graph_vertex_position = add_attribute<Vec3, GraphVertex>(*resampled_graph, "position");
			auto resampled_graph_vertex_radius = add_attribute<Scalar, GraphVertex>(*resampled_graph, "radius");

			modeling::resample_graph(*graph_, graph_vertex_position_.get(), graph_vertex_radius_.get(),
									 *resampled_graph, resampled_graph_vertex_position.get(),
									 resampled_graph_vertex_radius.get(), density);

			graph_provider_->emit_connectivity_changed(*graph_);
			graph_provider_->emit_attribute_changed(*graph_, graph_vertex_position_.get());
			graph_provider_->emit_attribute_changed(*graph_, graph_vertex_radius_.get());

			graph_provider_->emit_connectivity_changed(*resampled_graph);
			graph_provider_->emit_attribute_changed(*resampled_graph, resampled_graph_vertex_position.get());
			graph_provider_->emit_attribute_changed(*resampled_graph, resampled_graph_vertex_radius.get());

			return resampled_graph;
		}
		else
			return nullptr;
	}

	void subdivide_leaflets()
	{
		if constexpr (has_face<GRAPH>::value) // std::is_same_v<GRAPH, cgogn::IncidenceGraph>)
		{
			using Vertex = IncidenceGraph::Vertex;
			modeling::quadrangulate_all_faces(
				*graph_,
				[&](Vertex v) {
					std::vector<Vertex> av = adjacent_vertices_through_edge(*graph_, v);
					cgogn::value<Vec3>(*graph_, graph_vertex_position_, v) =
						0.5 * (cgogn::value<Vec3>(*graph_, graph_vertex_position_, av[0]) +
							   cgogn::value<Vec3>(*graph_, graph_vertex_position_, av[1]));
				},
				[&](Vertex v) {
					Vec3 center;
					center.setZero();
					uint32 count = 0;
					foreach_adjacent_vertex_through_edge(*graph_, v, [&](Vertex av) -> bool {
						center += cgogn::value<Vec3>(*graph_, graph_vertex_position_, av);
						++count;
						return true;
					});
					center /= Scalar(count);
					cgogn::value<Vec3>(*graph_, graph_vertex_position_, v) = center;
				});

			graph_provider_->emit_connectivity_changed(*graph_);
			graph_provider_->emit_attribute_changed(*graph_, graph_vertex_position_.get());
		}
	}

	VOLUME* build_hex_mesh()
	{
		// Scalar min_radius = std::numeric_limits<Scalar>::max();
		// for (Scalar r : *graph_vertex_radius_)
		// 	if (r < min_radius)
		// 		min_radius = r;

		// auto radius_copy = add_attribute<Scalar, GraphVertex>(*graph_, "radius_copy");
		// radius_copy->copy(graph_vertex_radius_.get());
		// graph_vertex_radius_->fill(min_radius);

		// for (Scalar& r : *graph_vertex_radius_)
		// 	r = r / 1.1;

		contact_surface_ = surface_provider_->add_mesh("contact");
		volume_ = volume_provider_->add_mesh("hex");

		if constexpr (std::is_same_v<GRAPH, Graph>)
			hex_building_attributes_ = modeling::graph_to_hex(*graph_, *contact_surface_, *volume_);

		if constexpr (std::is_same_v<GRAPH, IncidenceGraph>)
			hex_building_attributes_ig_ = modeling::incidenceGraph_to_hex(*graph_, *contact_surface_, *volume_);

		// if (!transversal_faces_marker_)
		// {
		// 	transversal_faces_marker_ = std::make_unique<CellMarker<VOLUME, VolumeFace>>(*volume_);
		// 	modeling::mark_tranversal_faces(*volume_, *contact_surface_, std::get<1>(hex_building_attributes_),
		// 									*transversal_faces_marker_);
		// }

		// graph_vertex_radius_->swap(radius_copy.get());
		// remove_attribute<GraphVertex>(*graph_, radius_copy);

		if (check_integrity(*volume_))
			std::cout << "Volume mesh OK!" << std::endl;

		surface_provider_->emit_connectivity_changed(*contact_surface_);
		volume_provider_->emit_connectivity_changed(*volume_);

		set_current_volume(volume_); // acquire attributes

		volume_provider_->set_mesh_bb_vertex_position(*volume_, volume_vertex_position_);

		graph_provider_->emit_connectivity_changed(*graph_);
		graph_provider_->emit_attribute_changed(*graph_, graph_vertex_position_.get());

		set_volume_caches_dirty();

		return volume_;
	}

	void add_volume_padding(bool pad_extremities)
	{
		if constexpr (std::is_same_v<GRAPH, Graph>)
			modeling::padding(*volume_, nullptr);

		if constexpr (std::is_same_v<GRAPH, IncidenceGraph>)
			modeling::padding(*volume_,
							  pad_extremities ? nullptr : std::get<2>(hex_building_attributes_ig_).extremity_faces);

		refresh_volume_skin();

		Scalar l = geometry::mean_edge_length(*graph_, graph_vertex_position_.get());
		parallel_foreach_cell(*volume_skin_, [&](SurfaceVertex v) -> bool {
			Vec3 n = geometry::normal(*volume_skin_, v, volume_skin_vertex_position_.get());
			Vec3 p = value<Vec3>(*volume_skin_, volume_skin_vertex_position_, v) + (0.2 * l) * n;
			value<Vec3>(*volume_skin_, volume_skin_vertex_position_, v) = p;
			value<Vec3>(*volume_, volume_vertex_position_,
						value<VolumeVertex>(*volume_skin_, volume_skin_vertex_volume_vertex_, v)) = p;
			return true;
		});

		volume_provider_->emit_connectivity_changed(*volume_);
		volume_provider_->emit_attribute_changed(*volume_, volume_vertex_position_.get());

		surface_provider_->emit_attribute_changed(*volume_skin_, volume_skin_vertex_position_.get());

		set_volume_caches_dirty(false);
	}

	void subdivide_slice()
	{
		if (selected_volume_faces_set_->size() == 1)
		{
			VolumeEdge e = modeling::find_fiber_dir(*volume_, *(selected_volume_faces_set_->begin()));
			CellCache<VOLUME> slice = modeling::get_slice(*volume_, e);
			modeling::cut_slice(*volume_, volume_vertex_position_.get(), slice);

			volume_provider_->emit_connectivity_changed(*volume_);
			volume_provider_->emit_attribute_changed(*volume_, volume_vertex_position_.get());

			set_volume_caches_dirty();
		}
	}

	void subdivide_all_slices()
	{
		if (selected_volume_faces_set_->size() == 1)
		{
			VolumeEdge e = modeling::find_fiber_dir(*volume_, *(selected_volume_faces_set_->begin()));
			CellCache<VOLUME> slices = modeling::surface_fiber_spread(*volume_, e);

			foreach_cell(slices, [&](VolumeEdge e) -> bool {
				CellCache<VOLUME> slice = modeling::get_slice(*volume_, e);
				modeling::cut_slice(*volume_, volume_vertex_position_.get(), slice);
				return true;
			});

			volume_provider_->emit_connectivity_changed(*volume_);
			volume_provider_->emit_attribute_changed(*volume_, volume_vertex_position_.get());

			set_volume_caches_dirty();
		}
	}

	void fiber_aligned_subdivision_from_input()
	{
		if (selected_volume_faces_set_->size() == 1)
		{
			CellMarker<VOLUME, VolumeEdge> edge_fibers(*volume_);
			VolumeEdge e = modeling::find_fiber_dir(*volume_, *(selected_volume_faces_set_->begin()));
			modeling::mark_mesh_fibers(*volume_, e, edge_fibers);

			modeling::fiber_aligned_subdivision(*volume_, edge_fibers);
			volume_provider_->emit_connectivity_changed(*volume_);
			volume_provider_->emit_attribute_changed(*volume_, volume_vertex_position_.get());

			set_volume_caches_dirty();
		}
	}

	void set_current_graph(GRAPH* g)
	{
		graph_ = g;
		graph_vertex_position_ = nullptr;
		graph_vertex_radius_ = nullptr;
	}

	void set_current_graph_vertex_position(const std::shared_ptr<GraphAttribute<Vec3>>& attribute)
	{
		if (graph_)
			graph_vertex_position_ = attribute;
	}

	void set_current_graph_vertex_radius(const std::shared_ptr<GraphAttribute<Scalar>>& attribute)
	{
		if (graph_)
			graph_vertex_radius_ = attribute;
	}

protected:
	void left_panel_meshes() override
	{
		ImGui::TextUnformatted("Graph");
		imgui_mesh_selector(graph_provider_, graph_, "Graph", [&](GRAPH& g) { set_current_graph(&g); });
		if (graph_)
		{
			imgui_combo_attribute<GraphVertex, Vec3>(*graph_, graph_vertex_position_, "Position##graph",
													 [&](const std::shared_ptr<GraphAttribute<Vec3>>& attribute) {
														 set_current_graph_vertex_position(attribute);
													 });
			imgui_combo_attribute<GraphVertex, Scalar>(*graph_, graph_vertex_radius_, "Radius##graph",
													   [&](const std::shared_ptr<GraphAttribute<Scalar>>& attribute) {
														   set_current_graph_vertex_radius(attribute);
													   });

			if (ImGui::Button("Create radius attribute"))
				graph_vertex_radius_ = add_attribute<Scalar, GraphVertex>(*graph_, "radius");
		}

		ImGui::Separator();
		Base::left_panel_meshes();
	}

	void left_panel_subdivision_operations(MeshData<VOLUME>& md) override
	{
		Base::left_panel_subdivision_operations(md);

		static bool pad_extremities = true;
		ImGui::Checkbox("Pad extremities", &pad_extremities);
		if (ImGui::Button("Add volume padding"))
			add_volume_padding(pad_extremities);

		imgui_combo_cells_set(md, selected_volume_faces_set_, "Faces",
								[&](CellsSet<VOLUME, VolumeFace>* cs) { selected_volume_faces_set_ = cs; });
		if (selected_volume_faces_set_)
		{
			if (ImGui::Button("Subdivide slice"))
				subdivide_slice();
			if (ImGui::Button("Subdivide all slices"))
				subdivide_all_slices();
		}
	}

	void left_panel_operations() override
	{
		ImGui::TextUnformatted("Graph Operations");
		if (graph_ && graph_vertex_position_ && graph_vertex_radius_)
		{
			if (ImGui::Button("Init radius from edge length"))
				init_graph_radius_from_edge_length();
			if (ImGui::Button("Subdivide leaflets"))
				subdivide_leaflets();
		}
		if (graph_ && graph_vertex_position_ && graph_vertex_radius_ && surface_ && surface_vertex_position_)
		{
			if (ImGui::Button("Init radius from surface"))
				init_graph_radius_from_surface();
			if (ImGui::Button("Recenter graph from surface"))
				recenter_graph_from_surface();
			if (ImGui::Button("Extend graph extremities"))
				extend_graph_extremities();
		}
		if (graph_ && graph_vertex_position_ && graph_vertex_radius_)
		{
			static float graph_resample_density = 0.5f;
			ImGui::SliderFloat("Graph resampling density", &graph_resample_density, 0.0, 2.0);
			if (ImGui::Button("Resample graph"))
				resample_graph(graph_resample_density);
		}

		ImGui::Separator();
		if (graph_ && graph_vertex_position_)
		{
			if (!volume_)
				if (ImGui::Button("Build hex mesh"))
					build_hex_mesh();
		}

		Base::left_panel_operations();
	}

private:
	MeshProvider<GRAPH>* graph_provider_ = nullptr;

	GRAPH* graph_ = nullptr;
	std::shared_ptr<GraphAttribute<Vec3>> graph_vertex_position_ = nullptr;
	std::shared_ptr<GraphAttribute<Scalar>> graph_vertex_radius_ = nullptr;

private:
	// std::unique_ptr<CellMarker<VOLUME, VolumeFace>> transversal_faces_marker_;
	CellsSet<VOLUME, VolumeFace>* selected_volume_faces_set_ = nullptr;

	std::tuple<modeling::GAttributes, modeling::M2Attributes, modeling::M3Attributes> hex_building_attributes_;
	std::tuple<modeling::IG_GAttributes, modeling::IG_M2Attributes, modeling::IG_M3Attributes>
		hex_building_attributes_ig_;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_TUBULAR_MESH_H_
