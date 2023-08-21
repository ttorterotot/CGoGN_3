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

#ifndef CGOGN_MODULE_POWER_SHAPE_H_
#define CGOGN_MODULE_POWER_SHAPE_H_

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>
#include <cgogn/core/functions/mesh_ops/edge.h>
#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn\modeling\algos\decimation\SQEM_helper.h>
#include <cgogn/io/point/point_import.h>
#include <cgogn/geometry/types/slab_quadric.h>
// import CGAL
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Object.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Regular_triangulation_cell_base_3.h>
#include <CGAL/Regular_triangulation_vertex_base_3.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/double.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Bbox_3.h>
#include <Highs.h>

namespace cgogn
{

namespace ui
{

template <typename POINT, typename SURFACE, typename NONMANIFOLD>
class PowerShape : public Module
{
	static_assert(mesh_traits<SURFACE>::dimension >= 2, "PowerShape can only be used with meshes of dimension >= 2");

	// Kernel for construct Delaunay
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Vb = CGAL::Triangulation_vertex_base_with_info_3<uint32_t, K>;
	// Delaunay
	using Cb = CGAL::Delaunay_triangulation_cell_base_3<K>;
	using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
	using Delaunay = CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location>;
	// Regular
	using Vb0 = CGAL::Regular_triangulation_vertex_base_3<K>;
	using RVb = CGAL::Triangulation_vertex_base_with_info_3<std::pair<uint32_t, bool>, K, Vb0>;
	using RCb = CGAL::Regular_triangulation_cell_base_3<K>;
	using RTds = CGAL::Triangulation_data_structure_3<RVb, RCb>;
	using Regular = CGAL::Regular_triangulation_3<K, RTds, CGAL::Fast_location>;

	using Point = K::Point_3;
	using Weight = K::FT;
	using Weight_Point = K::Weighted_point_3;

	using Cgal_Surface_mesh = CGAL::Surface_mesh<Point>;
	using Point_inside = CGAL::Side_of_triangle_mesh<Cgal_Surface_mesh, K>;
	using Primitive = CGAL::AABB_face_graph_triangle_primitive<Cgal_Surface_mesh>;
	using Traits = CGAL::AABB_traits<K, Primitive>;
	using Tree = CGAL::AABB_tree<Traits>;

	template <typename T>
	using NonManifoldAttribute = typename mesh_traits<NONMANIFOLD>::template Attribute<T>;
	template <typename T>
	using SurfaceAttribute = typename mesh_traits<SURFACE>::template Attribute<T>;
	using PointVertex = typename mesh_traits<POINT>::Vertex; 
	using NonManifoldVertex = typename mesh_traits<NONMANIFOLD>::Vertex;
	using NonManifoldEdge = typename mesh_traits<NONMANIFOLD>::Edge;
	using NonManifoldFace = typename mesh_traits<NONMANIFOLD>::Face;

	using SurfaceVertex = typename mesh_traits<SURFACE>::Vertex;
	using SurfaceEdge = typename mesh_traits<SURFACE>::Edge;
	using SurfaceFace = typename mesh_traits<SURFACE>::Face;

	using Vec4 = geometry::Vec4;
	using Vec3 = geometry::Vec3;
	using Scalar = geometry::Scalar;

public:
	PowerShape(const App& app)
		: Module(app, "PowerShape"), selected_surface_mesh_(nullptr)
	{
	}
	~PowerShape()
	{
	}

private:
	
	struct point_hash
	{
		std::size_t operator()(const Point& p) const
		{
			return ((std::hash<double>()(p.x()) ^ (std::hash<double>()(p.y()) << 1)) >> 1) ^
				   (std::hash<double>()(p.z()) << 1);
		}
	};

	struct edge_hash
	{
		std::size_t operator()(const std::pair<uint32, uint32>& edge) const
		{
			return std::hash<uint32>()(edge.first) + std::hash<uint32>()(edge.second);
		}
	};
	struct edge_equal
	{
		bool operator()(const std::pair<uint32, uint32>& edge1, const std::pair<uint32, uint32>& edge2) const
		{
			return ((edge1.first == edge2.first && edge1.second == edge2.second) ||
					(edge1.first == edge2.second && edge1.second == edge2.first));
		}
	};
	bool pointInside(Tree& tree, Point& query)
	{
		// Initialize the point-in-polyhedron tester
		Point_inside inside_tester(tree);

		// Determine the side and return true if inside!
		return inside_tester(query) == CGAL::ON_BOUNDED_SIDE;
	}

public:

	Delaunay compute_delaunay_tredrahedron(SURFACE& surface, Cgal_Surface_mesh& csm, Tree& tree)
	{
		cgogn::io::PointImportData samples;
		std::vector<Point> Delaunay_tri_point;
		std::array<Point, 8> obb_points;
		Point acc(0, 0, 0);
		CGAL::oriented_bounding_box(csm, obb_points, CGAL::parameters::use_convex_hull(true));
		for (size_t i = 0; i < obb_points.size(); i++)
		{
			acc += K::Vector_3(obb_points[i].x(), obb_points[i].y(), obb_points[i].z());
		}
		std::array<double, 3> center{acc.x() / 8, acc.y() / 8, acc.z() / 8};
		// Create a large box surrounding object so that the Voronoi vertices are bounded
		MeshData<SURFACE>& md = surface_provider_->mesh_data(surface);
		double offset = (md.bb_max_ - md.bb_min_).norm() *20.0;
		std::array<double, 3> offset_array = {offset, offset, offset};
		std::array<std::array<double, 3>, 8> cube_corners = {
			{{center[0] - offset_array[0], center[1] - offset_array[1], center[2] - offset_array[2]},
			 {center[0] - offset_array[0], center[1] - offset_array[1], center[2] + offset_array[2]},
			 {center[0] - offset_array[0], center[1] + offset_array[1], center[2] - offset_array[2]},
			 {center[0] - offset_array[0], center[1] + offset_array[1], center[2] + offset_array[2]},
			 {center[0] + offset_array[0], center[1] - offset_array[1], center[2] - offset_array[2]},
			 {center[0] + offset_array[0], center[1] - offset_array[1], center[2] + offset_array[2]},
			 {center[0] + offset_array[0], center[1] + offset_array[1], center[2] - offset_array[2]},
			 {center[0] + offset_array[0], center[1] + offset_array[1], center[2] + offset_array[2]}}};

		// Sampling the mesh surface
		std::vector<Point> mesh_samples;
		CGAL::Polygon_mesh_processing::sample_triangle_mesh(
			csm, std::back_inserter(mesh_samples),
		    //CGAL::parameters::use_monte_carlo_sampling(true).number_of_points_per_area_unit(50));
			CGAL::parameters::use_grid_sampling(true).grid_spacing(1));

		// 	Add bounding box vertices in the sample points set
		for (auto& p : cube_corners)
		{
			Delaunay_tri_point.emplace_back(p[0], p[1], p[2]);
			//samples.vertex_position_.emplace_back(p[0], p[1], p[2]); //Only for debugging
		}

		// Add sampled vertices into the volume data to construct the delauney tredrahedron
		for (auto& s : mesh_samples)
		{
			Delaunay_tri_point.emplace_back(s[0], s[1], s[2]);
			samples.vertex_position_.emplace_back(s[0], s[1], s[2]);
		}
		//uint32 nb_vertices = Delaunay_tri_point.size(); //Only for debugging
		uint32 nb_vertices = mesh_samples.size();

		//		auto start_timer = std::chrono::high_resolution_clock::now();

		// Indices info for constructing volume data in CGogn
		std::vector<unsigned> indices;
		indices.reserve(Delaunay_tri_point.size());
		for (unsigned int i = 0; i < Delaunay_tri_point.size(); ++i)
			indices.push_back(i);
		//Collect point data
		samples.reserve(nb_vertices);
		cgogn::io::import_point_data(*surface_sample, samples);
		auto position = get_attribute<Vec3, PointVertex>(*surface_sample, "position");
		if (position)
			point_provider_->set_mesh_bb_vertex_position(*surface_sample, position);
		// Construct delauney tredrahedron using CGAL
		Delaunay tri(boost::make_zip_iterator(boost::make_tuple(Delaunay_tri_point.begin(), indices.begin())),
					 boost::make_zip_iterator(boost::make_tuple(Delaunay_tri_point.end(), indices.end())));
		return tri;
	}
	
	void compute_initial_non_manifold(Delaunay& tri, Tree& tree, NONMANIFOLD* mv)
	{
		std::vector<double> vector_sphere_radius;
		cgogn::io::IncidenceGraphImportData Initial_non_manifold;
		std::unordered_map<Point, uint32, point_hash> vertex_indices;
		std::unordered_map<Delaunay::Cell_handle, Point> cell_vertex_correspondence;
		std::unordered_map<std::pair<uint32, uint32>, uint32, edge_hash, edge_equal> edge_indices;

		uint32 vertex_count = 0, edge_count = 0;
		// Add vertices
		for (auto cit = tri.finite_cells_begin(); cit != tri.finite_cells_end(); ++cit)
		{
			Point centroid = CGAL::circumcenter(tri.tetrahedron(cit));
			double radius = CGAL::squared_distance(centroid, cit->vertex(0)->point());
			if (pointInside(tree, centroid))
			{
				Initial_non_manifold.vertex_position_.emplace_back(centroid[0], centroid[1], centroid[2]);
				vertex_indices.insert({centroid, vertex_count});
				cell_vertex_correspondence.insert({cit, centroid});
				vector_sphere_radius.push_back(std::sqrt(radius));
				vertex_count++;
			}
		}
		//Add edges
		for (auto fit = tri.finite_facets_begin(); fit != tri.finite_facets_end(); ++fit)
		{
			Delaunay::Object o = tri.dual(*fit);
			
			if (const Delaunay::Segment* s = CGAL::object_cast<Delaunay::Segment>(&o))
			{
				Point p1 = s->point(0);
				Point p2 = s->point(1);
				if (pointInside(tree, p1) && pointInside(tree, p2))
				{
					Initial_non_manifold.edges_vertex_indices_.push_back(vertex_indices[p1]);
					Initial_non_manifold.edges_vertex_indices_.push_back(vertex_indices[p2]);
					edge_indices.insert({{vertex_indices[p1], vertex_indices[p2]}, edge_count});
					edge_count++;
				}
			}
		}
		bool all_finite_inside;
		std::vector<Delaunay::Cell_handle> incells;
		for (auto eit = tri.finite_edges_begin(); eit != tri.finite_edges_end(); ++eit) {
			all_finite_inside = true;
			incells.clear();
			Delaunay::Cell_circulator cc = tri.incident_cells(*eit);
			do {
				if (tri.is_infinite(cc) || cell_vertex_correspondence.find(cc) == cell_vertex_correspondence.end())
				{
					all_finite_inside = false;
					break;
				}
				if (!pointInside(tree, cell_vertex_correspondence[cc]))
				{
					all_finite_inside = false;
					break;
				}
				incells.push_back(cc);
			} while (++cc != tri.incident_cells(*eit));
			if (!all_finite_inside)
				continue;
			for (size_t k = 2; k < incells.size() - 1; ++k)
			{
				auto c1 = incells[0];
				auto c2 = incells[k];
				uint32 ev1 = vertex_indices[cell_vertex_correspondence[c1]];
				uint32 ev2 = vertex_indices[cell_vertex_correspondence[c2]];
				// Check if the edge is already added
				if (edge_indices.find({ev1,ev2})!=edge_indices.end())
					continue;
				Initial_non_manifold.edges_vertex_indices_.push_back(ev1);
				Initial_non_manifold.edges_vertex_indices_.push_back(ev2);
				edge_indices.insert({{ev1,ev2}, edge_count});
				edge_count++;

			}
			for (size_t k = 1; k < incells.size() - 1; ++k)
			{
				uint32 v1 = vertex_indices[cell_vertex_correspondence[incells[0]]];
				uint32 v2 = vertex_indices[cell_vertex_correspondence[incells[k]]];
				uint32 v3 = vertex_indices[cell_vertex_correspondence[incells[k + 1]]];
				uint32 e1 = edge_indices[{v1,v2}];
				uint32 e2 = edge_indices[{v2,v3}];
				uint32 e3 = edge_indices[{v3,v1}];
				Initial_non_manifold.faces_nb_edges_.push_back(3);
				Initial_non_manifold.faces_edge_indices_.push_back(e1);
				Initial_non_manifold.faces_edge_indices_.push_back(e2);
				Initial_non_manifold.faces_edge_indices_.push_back(e3);
			}
		}
		uint32 Initial_non_manifold_nb_vertices = Initial_non_manifold.vertex_position_.size();
		uint32 Initial_non_manifold_nb_edges = Initial_non_manifold.edges_vertex_indices_.size() / 2;
		uint32 Initial_non_manifold_nb_faces = Initial_non_manifold.faces_nb_edges_.size();
		Initial_non_manifold.set_parameter(Initial_non_manifold_nb_vertices, Initial_non_manifold_nb_edges,
											 Initial_non_manifold_nb_faces);

		import_incidence_graph_data(*mv, Initial_non_manifold);
		auto sphere_raidus = add_attribute<double, NonManifoldVertex>(*mv, "sphere_radius");
		for (auto it = vertex_indices.begin(); it != vertex_indices.end(); ++it)
		{
			(*sphere_raidus)[it->second] = vector_sphere_radius[it->second];
		}
	
		std::shared_ptr<NonManifoldAttribute<Vec3>> mv_vertex_position =
			get_attribute<Vec3, NonManifoldVertex>(*mv, "position");
		if (mv_vertex_position)
			nonmanifold_provider_->set_mesh_bb_vertex_position(*mv, mv_vertex_position);

		nonmanifold_provider_->emit_connectivity_changed(*mv);
	}

	void compute_inner_voronoi(Delaunay& tri, Tree& tree, std::vector<Weight_Point>& power_point,
							   std::vector<std::pair<uint32, bool>>& point_info,
							   std::unordered_map<uint32, uint32>& inside_indices)
	{
		uint32 count = 0, inside_vertices_count = 0;
		double dis;
		for (auto cit = tri.finite_cells_begin(); cit != tri.finite_cells_end(); ++cit)
		{
			Point centroid = CGAL::circumcenter(tri.tetrahedron(cit));
			dis = CGAL::squared_distance(centroid, cit->vertex(0)->point());
			power_point.push_back(Weight_Point(centroid, dis));
			if (pointInside(tree, centroid))
			{
				point_info.push_back({count, true});
				inside_indices.insert({count, inside_vertices_count});
				count++;
				inside_vertices_count++;
			}
			else if (!pointInside(tree, centroid))
			{
				point_info.push_back({count, false});
				count++;
			}
		}
	}

	void compute_inner_poles(Delaunay& tri, Tree& tree, std::vector<Weight_Point>& power_point,
							 std::vector<std::pair<uint32, bool>>& point_info,
							 std::unordered_map<uint32, uint32>& inside_indices)
	{
		std::vector<std::pair<uint32, bool>> power_indices;
		// Find inside and outside poles
		uint32 count = 0, inside_poles_count = 0;
		double dis;
		Point farthest_inside_point, farthest_outside_point;
		double farthest_inside_distance, farthest_outside_distance;
		std::unordered_set<Point, point_hash> poles;
		 // Use for the construction of medial axis
		std::vector<Delaunay::Cell_handle> cells;
		for (auto vit = tri.finite_vertices_begin(); vit != tri.finite_vertices_end(); ++vit)
		{
			farthest_inside_distance = 0;
			farthest_outside_distance = 0;
			cells.clear();
			tri.finite_incident_cells(vit, std::back_inserter(cells));
			if (cells.size())
			{
				for (auto c = cells.begin(); c != cells.end(); ++c)
				{
					Point centroid = tri.dual(*c);
					dis = CGAL::squared_distance(centroid, vit->point());
					if (pointInside(tree, centroid) && dis > farthest_inside_distance)
					{
						farthest_inside_point = centroid;
						farthest_inside_distance = dis;
					}
					else if (!pointInside(tree, centroid) && dis > farthest_outside_distance)
					{
						farthest_outside_point = centroid;
						farthest_outside_distance = dis;
					}
				}
				if (farthest_inside_distance != 0)
				{
					if (poles.find(farthest_inside_point) == poles.end())
					{
						poles.insert(farthest_inside_point);
						power_point.push_back(Weight_Point(farthest_inside_point, farthest_inside_distance));
						point_info.push_back({count, true});
						inside_indices.insert({count, inside_poles_count});
						count++;
						inside_poles_count++;
					}
				}
				if (farthest_outside_distance != 0)
				{
					if (poles.find(farthest_outside_point) == poles.end())
					{
						poles.insert(farthest_outside_point);
						power_point.push_back(Weight_Point(farthest_outside_point, farthest_outside_distance));
						point_info.push_back({count, false});
						count++;
					}
				}
			}
			
		}
	}
	/* void construct_complete_power_diagram(NONMANIFOLD* mv, std::vector<Weight_Point>& power_point,
										  std::vector<std::pair<uint32, bool>>& point_info)
	{
		cgogn::io::IncidenceGraphImportData Complete_Power_shape_data;

		std::unordered_map<std::pair<uint32, uint32>, uint32, edge_hash, edge_equal> edge_indices;
		uint32 edge_count = 0;

		medial_axis = Regular(boost::make_zip_iterator(boost::make_tuple(power_point.begin(), point_info.begin())),
							  boost::make_zip_iterator(boost::make_tuple(power_point.end(), point_info.end())));

		for (size_t idx = 0; idx < power_point.size(); ++idx)
		{
			Complete_Power_shape_data.vertex_position_.push_back(
				{power_point[idx].x(), power_point[idx].y(), power_point[idx].z()});
		}
		uint32 v, v1, v2;
		for (auto fit = medial_axis.finite_facets_begin(); fit != medial_axis.finite_facets_end(); ++fit)
		{
			v = fit->second;
			Complete_Power_shape_data.faces_nb_edges_.push_back(3);
			for (size_t i = 0; i < 4; i++)
			{
				if (i != v)
				{
					for (size_t j = i + 1; j < 4; j++)
					{
						if (j != v)
						{
							// Add edge
							v1 = fit->first->vertex(i)->info().first;
							v2 = fit->first->vertex(j)->info().first;
							if (edge_indices.find({v1, v2}) == edge_indices.end())
							{
								edge_indices.insert({{v1, v2}, edge_count});
								Complete_Power_shape_data.edges_vertex_indices_.push_back(v1);
								Complete_Power_shape_data.edges_vertex_indices_.push_back(v2);
								edge_count++;
							}
							// Add face
							Complete_Power_shape_data.faces_edge_indices_.push_back(edge_indices[{v1, v2}]);
						}
					}
				}
			}
			
		}
		uint32 complete_power_nb_vertices = Complete_Power_shape_data.vertex_position_.size();
		uint32 complete_power_nb_edges = Complete_Power_shape_data.edges_vertex_indices_.size() / 2;
		uint32 complete_power_nb_faces = Complete_Power_shape_data.faces_nb_edges_.size()/3;

		Complete_Power_shape_data.set_parameter(complete_power_nb_vertices, complete_power_nb_edges,
												complete_power_nb_faces);

		import_incidence_graph_data(*mv, Complete_Power_shape_data);
		auto sphere_raidus = add_attribute<double, NonManifoldVertex>(*mv, "sphere_radius");
		for (uint32 i = 0u; i < power_point.size(); ++i)
		{
			uint32 vertex_id = point_info[i].first;
			// The radius is sqrt of the weight!
			(*sphere_raidus)[vertex_id] = std::sqrt(power_point[i].weight());
			
		}

		std::shared_ptr<NonManifoldAttribute<Vec3>> mv_vertex_position =
			get_attribute<Vec3, NonManifoldVertex>(*mv, "position");
		if (mv_vertex_position)
			nonmanifold_provider_->set_mesh_bb_vertex_position(*mv, mv_vertex_position);

		nonmanifold_provider_->emit_connectivity_changed(*mv);
	}*/
	void constrcut_inner_power_diagram(NONMANIFOLD* mv, std::vector<Weight_Point>& power_point,
									   std::vector<std::pair<uint32, bool>>& point_info,
									   std::unordered_map<uint32, uint32>& inside_indices)
	{
		cgogn::io::IncidenceGraphImportData Inner_Power_shape_data;

		std::unordered_map<std::pair<uint32, uint32>, uint32, edge_hash, edge_equal> edge_indices;
		uint32 edge_count = 0;
		medial_axis = Regular(boost::make_zip_iterator(boost::make_tuple(power_point.begin(), point_info.begin())),
							  boost::make_zip_iterator(boost::make_tuple(power_point.end(), point_info.end())));

		for (size_t idx = 0; idx < power_point.size(); ++idx)
		{
			// if the point is inside
			if (point_info[idx].second)
			{
				Inner_Power_shape_data.vertex_position_.push_back(
					{power_point[idx].x(), power_point[idx].y(), power_point[idx].z()});
			}
		}
		bool inside;
		uint32 v, v_ind1, v_ind2, v1, v2;
		for (auto eit = medial_axis.finite_edges_begin(); eit != medial_axis.finite_edges_end(); ++eit)
		{
			v_ind1 = eit->second;
			v_ind2 = eit->third;
			inside = eit->first->vertex(v_ind1)->info().second && eit->first->vertex(v_ind2)->info().second;
			if (inside)
			{
				//Add edge
				v1 = inside_indices[eit->first->vertex(v_ind1)->info().first];
				v2 = inside_indices[eit->first->vertex(v_ind2)->info().first];
				edge_indices.insert({{v1, v2}, edge_count}); 
				Inner_Power_shape_data.edges_vertex_indices_.push_back(v1);
				Inner_Power_shape_data.edges_vertex_indices_.push_back(v2);
				edge_count++;
				
			}
		}
	
		for (auto fit = medial_axis.finite_facets_begin(); fit != medial_axis.finite_facets_end(); ++fit)
		{
			inside = true;
			v = fit->second;
			// If face is inside
			for (size_t idx = 0; idx < 4; ++idx)
			{
				if (idx != v)
				{
					inside &= fit->first->vertex(idx)->info().second;
				}
			}
			if (inside)
			{
				Inner_Power_shape_data.faces_nb_edges_.push_back(3);
				for (size_t i = 0; i < 4; i++)
				{
					if (i != v)
					{
						for (size_t j = i + 1; j < 4; j++)
						{
							if (j != v)
							{
								v1 = inside_indices[fit->first->vertex(i)->info().first];
								v2 = inside_indices[fit->first->vertex(j)->info().first];
								Inner_Power_shape_data.faces_edge_indices_.push_back(edge_indices[{v1, v2}]);
							}
						}
					}
				}
			}
		}
		uint32 inner_power_nb_vertices = Inner_Power_shape_data.vertex_position_.size();
		uint32 inner_power_nb_edges = Inner_Power_shape_data.edges_vertex_indices_.size() / 2;
		uint32 inner_power_nb_faces = Inner_Power_shape_data.faces_nb_edges_.size();

		Inner_Power_shape_data.set_parameter(inner_power_nb_vertices, inner_power_nb_edges, inner_power_nb_faces);

		import_incidence_graph_data(*mv, Inner_Power_shape_data);
		auto sphere_raidus = add_attribute<double, NonManifoldVertex>(*mv, "sphere_radius");
		for (uint32 i = 0u; i < power_point.size(); ++i)
		{
			if (point_info[i].second)
			{
				uint32 vertex_id = inside_indices[point_info[i].first];
				// The radius is sqrt of the weight!
				(*sphere_raidus)[vertex_id] = std::sqrt(power_point[i].weight());
				//std::cout<< "vertex id: " << vertex_id << " radius: " << (*sphere_raidus)[vertex_id] << std::endl;
			}
		}

		std::shared_ptr<NonManifoldAttribute<Vec3>> mv_vertex_position =
			get_attribute<Vec3, NonManifoldVertex>(*mv, "position");
		if (mv_vertex_position)
			nonmanifold_provider_->set_mesh_bb_vertex_position(*mv, mv_vertex_position);

		nonmanifold_provider_->emit_connectivity_changed(*mv);
	}
	
	void compute_power_shape(SURFACE& surface)
	{
		surface_sample = point_provider_->add_mesh(point_provider_->mesh_name(surface) + "surface_samples");
		NONMANIFOLD* mv = nonmanifold_provider_->add_mesh(surface_provider_->mesh_name(surface) + "_inner_power_shape");
		//NONMANIFOLD* mp = nonmanifold_provider_->add_mesh(surface_provider_->mesh_name(surface) + "_complete_power_shape");
		Cgal_Surface_mesh csm;

		std::string filename = surface_provider_->mesh_filename(surface);
		if (!filename.empty())
		{
			std::ifstream input(filename);
			if (!input || !(input >> csm))
			{
				std::cerr << "Error: input file could not be read" << std::endl;
				return;
			}
		}
		//mesh_normalisation(csm);
		Tree tree(faces(csm).first, faces(csm).second, csm);
		tree.accelerate_distance_queries();
		Delaunay tri = compute_delaunay_tredrahedron(surface, csm, tree);
// 		std::vector<Weight_Point> Power_point;
// 		std::vector<std::pair<uint32, bool>> Point_info;
// 		std::unordered_map<uint32, uint32> Inside_indices;
// 		compute_inner_poles(tri, tree, Power_point, Point_info, Inside_indices);
		//constrcut_inner_power_diagram(mv, Power_point, Point_info, Inside_indices);
		compute_initial_non_manifold(tri, tree, mv);
		//construct_complete_power_diagram(mp, Power_point, Point_info);
	}

	void compute_original_power_diagram(SURFACE& surface)
	{
		surface_sample = point_provider_->add_mesh(point_provider_->mesh_name(surface) + "surface_samples");
		NONMANIFOLD* mv = nonmanifold_provider_->add_mesh(surface_provider_->mesh_name(surface) + "_inner_medial_axis");
		//NONMANIFOLD* mp = nonmanifold_provider_->add_mesh(surface_provider_->mesh_name(surface) + "_complete_medial_axis");
		Cgal_Surface_mesh csm;

		std::string filename = surface_provider_->mesh_filename(surface);
		if (!filename.empty())
		{
			std::ifstream input(filename);
			if (!input || !(input >> csm))
			{
				std::cerr << "Error: input file could not be read" << std::endl;
				return;
			}
		}
		//mesh_normalisation(csm);
		Tree tree(faces(csm).first, faces(csm).second, csm);
		tree.accelerate_distance_queries();
		Delaunay tri = compute_delaunay_tredrahedron(surface,csm, tree);
// 		std::vector<Weight_Point> Power_point;
// 		std::vector<std::pair<uint32, bool>> Point_info;
// 		std::unordered_map<uint32, uint32> Inside_indices;
// 		compute_inner_voronoi(tri, tree, Power_point, Point_info, Inside_indices);
		compute_initial_non_manifold(tri, tree, mv);
 		//constrcut_inner_power_diagram(mv, Power_point, Point_info, Inside_indices);
		//construct_complete_power_diagram(mp, Power_point, Point_info);
	}

	bool inside_sphere(const Vec3& point, const Vec3& center, double radius)
	{
		return (point - center).norm() <= radius;
	}

	void compute_stability_ratio_edge(NONMANIFOLD& nm, NonManifoldEdge e, 
		std::shared_ptr<NonManifoldAttribute<double>>& stability_ratio,
									  std::shared_ptr<NonManifoldAttribute<Vec3>>& stability_color,
									  std::shared_ptr<NonManifoldAttribute<double>>& sphere_radius,
									  std::shared_ptr<NonManifoldAttribute<Vec3>>& position,
		)
	{
		auto iv = incident_vertices(nm, e);
		NonManifoldVertex v1 = iv[0];
		NonManifoldVertex v2 = iv[1];
		const Vec3& v1_p = value<Vec3>(nm, position, v1);
		const Vec3& v2_p = value<Vec3>(nm, position, v2);
		const double& r1 = value<double>(nm, sphere_radius, v1);
		const double& r2 = value<double>(nm, sphere_radius, v2);
		const double center_dist = (v1_p - v2_p).norm();
		double dis = std::max(0.0, (center_dist - std::abs(r1 - r2)));
		double stability = dis / center_dist;
		// std::cout << "Edge: " << e.index_ << ", Stability ratio: " << stability << std::flush;
		(*stability_ratio)[e.index_] = stability;
		if (stability <= 0.5)
		{
			(*stability_color)[e.index_] = Vec3(0, stability, (0.5 - stability));
		}
		else
		{
			(*stability_color)[e.index_] = Vec3(stability - 0.5, (1 - stability), 0);
		}
	}

 	void compute_stability_ratio(NONMANIFOLD& nm)
	{
		auto stability_ratio = get_attribute<double, NonManifoldEdge>(nm, "stability_ratio");
		auto stability_color = get_attribute<Vec3, NonManifoldEdge>(nm, "stability_color");
		auto sphere_radius = get_attribute<double, NonManifoldVertex>(nm, "sphere_radius");
		auto position = get_attribute<Vec3, NonManifoldVertex>(nm, "position");
		parallel_foreach_cell(nm, [&](NonManifoldEdge e) -> bool { 
			compute_stability_ratio_edge(nm, e,stability_ratio, stability_color, sphere_radius, position);
			return true;
		});
	}

	 void collapse_non_manifold_using_QMat(NONMANIFOLD& nm, uint32 number_vertices_erase, double k)
	{
		using EdgeQueue = std::multimap<Scalar, NonManifoldEdge>;
		using EdgeQueueIt = typename EdgeQueue::const_iterator;
		
		using EdgeInfo = std::pair<bool, EdgeQueueIt>; // {valid, iterator}
		using QMatHelper = modeling::DecimationSQEM_Helper<NONMANIFOLD>;
		using Slab_Quadric = geometry::Slab_Quadric;
		uint32 count = 0;
		EdgeQueue queue;

		auto stability_ratio = add_attribute<double, NonManifoldEdge>(nm, "stability_ratio");
		auto stability_color = add_attribute<Vec3, NonManifoldEdge>(nm, "stability_color");
		auto edge_queue_it = add_attribute<EdgeInfo, NonManifoldEdge>(nm, "__non_manifold_edge_queue_it");
		auto sphere_info = add_attribute<Vec4, NonManifoldVertex>(nm, "sphere_info");
		auto sphere_opt = add_attribute<Vec4, NonManifoldEdge>(nm, "sphere_opt");
		auto slab_quadric = add_attribute<Slab_Quadric, NonManifoldVertex>(nm, "__slab_quadric");

		compute_stability_ratio(nm);

		auto position = get_attribute<Vec3, NonManifoldVertex>(nm, "position");
		auto sphere_radius = get_attribute<double, NonManifoldVertex>(nm, "sphere_radius");
		
		//build sphere info
		foreach_cell(nm,
					 [&](NonManifoldVertex v) {
						 Vec3 p = value<Vec3>(nm, position, v);
			value<Vec4>(nm, sphere_info, v) = Vec4(p[0], p[1], p[2], value<double>(nm,sphere_radius,v));
			return true;
		});
		QMatHelper helper(nm, sphere_info, slab_quadric); 
		
		//Initialize the queue with all the edges and their cost
		foreach_cell(nm, [&](NonManifoldEdge e) -> bool {
			Vec4 opt = helper.edge_optimal(e);
			value<Vec4>(nm, sphere_opt, e) = opt;
			double cost = helper.edge_cost(e, opt);
			double cost_opt =
				(cost + k) * value<double>(nm, stability_ratio, e) * value<double>(nm, stability_ratio, e);
			value<EdgeInfo>(nm, edge_queue_it, e) = {
				true, 
				queue.emplace(cost_opt,e)};
		
			return true;
		});

		 while (!queue.empty() && count < number_vertices_erase)
		{
			auto it = queue.begin();
			NonManifoldEdge e = (*it).second;
			queue.erase(it);
			value<EdgeInfo>(nm, edge_queue_it, e).first = false;

			auto iv = incident_vertices(nm, e);
			NonManifoldVertex v1 = iv[0];
			NonManifoldVertex v2 = iv[1];
			Slab_Quadric quad = value<Slab_Quadric>(nm, slab_quadric, v1) + value<Slab_Quadric>(nm, slab_quadric, v2);
			
			auto [v, removed_edges] = collapse_edge_qmat(nm, e);
			value<Vec4>(nm, sphere_info, v) = value<Vec4>(nm, sphere_opt, e);
			value<Slab_Quadric>(nm, slab_quadric, v) = quad;
			for (NonManifoldEdge re : removed_edges)
			{
				EdgeInfo einfo = value<EdgeInfo>(nm, edge_queue_it, re);
				if (einfo.first)
					queue.erase(einfo.second);
			}

			//recompute the cost of the edges incident to v and update the queue
			foreach_incident_edge(nm, v, [&](NonManifoldEdge ie) -> bool {

				compute_stability_ratio_edge(nm, e, stability_ratio, stability_color, sphere_radius, position);

				//recompute the cost of the edge
				Vec4 opt = helper.edge_optimal(ie);
				value<Vec4>(nm, sphere_opt, ie) = opt;
				double cost = helper.edge_cost(ie, opt);
				double cost_opt =
					(cost + k) * value<double>(nm, stability_ratio, ie) * value<double>(nm, stability_ratio, ie);
				value<EdgeInfo>(nm, edge_queue_it, ie) = {true, queue.emplace(cost, ie)};
				return true;
			});
			++count;
		}

		foreach_cell(nm, [&](NonManifoldVertex v) -> bool {
			value<Vec3>(nm, position, v) = value<Vec4>(nm, sphere_info, v).head<3>();
			return true;
		});

		remove_attribute<NonManifoldEdge>(nm, edge_queue_it);
		remove_attribute<NonManifoldVertex>(nm, sphere_info);
		remove_attribute<NonManifoldEdge>(nm, sphere_opt);
		remove_attribute<NonManifoldVertex>(nm, slab_quadric);
		remove_attribute<NonManifoldEdge>(nm, stability_ratio);
		remove_attribute<NonManifoldEdge>(nm, stability_color);

	
		nonmanifold_provider_->emit_connectivity_changed(nm);
		nonmanifold_provider_->emit_attribute_changed(nm, position.get());
		nonmanifold_provider_->emit_attribute_changed(nm, sphere_radius.get());
		
	}


	void coverage_axis_PD(SURFACE& surface, NONMANIFOLD& mv, HighsSolution& solution, double dilation_factor)
	{
		auto inner_position = get_attribute<Vec3, NonManifoldVertex>(mv, "position");
		auto sample_position = get_attribute<Vec3, Vertex>(surface, "position");
		auto sphere_radius = get_attribute<double, NonManifoldVertex>(mv, "sphere_radius");
		std::vector<Weight_Point> power_point;
		std::vector<std::pair<uint32, bool>> point_info;
		std::unordered_map<uint32, uint32> inside_indices;
		uint32 count = 0;
		foreach_cell(mv, [&](NonManifoldVertex nv) {
			if (solution.col_value[index_of(mv, nv)] > 1e-5)
			{
				Vec3 pos = value<Vec3>(mv, inner_position, nv);
				power_point.push_back(Weight_Point(Point(pos[0], pos[1], pos[2]),
												   (value<double>(mv, sphere_radius, nv) + dilation_factor) *
													   (value<double>(mv, sphere_radius, nv) + dilation_factor)));
				point_info.push_back({count, true});
				inside_indices[count] = count;
				count++;
			}
			return true;
		});

		foreach_cell(surface, [&](Vertex v) {
			Vec3 pos = value<Vec3>(surface, sample_position, v);
			power_point.push_back(Weight_Point(Point(pos[0], pos[1], pos[2]), dilation_factor * dilation_factor));
			point_info.push_back({count, false});
			count++;
			return true;
		});

		NONMANIFOLD* ca = nonmanifold_provider_->add_mesh(surface_provider_->mesh_name(surface) + "_coverage_axis");
		constrcut_inner_power_diagram(ca, power_point, point_info, inside_indices);
	}

	void coverage_axis_collapse(NONMANIFOLD& nm, std::vector<double> selection_points)
	{
		using EdgeQueue = std::set<NonManifoldEdge>;
		using EdgeQueueIt = typename EdgeQueue::const_iterator;
		EdgeQueue queue;
		auto edge_queue_it = add_attribute<EdgeQueueIt, NonManifoldEdge>(nm, "__non_manifold_edge_queue_it");
		foreach_cell(nm, [&](NonManifoldEdge e) -> bool {
			value<EdgeQueueIt>(nm, edge_queue_it, e) = queue.emplace(e).first;
			return true;
		});
		while (!queue.empty())
		{
			auto it = queue.begin();
			NonManifoldEdge e = (*it);
			queue.erase(it);
			auto removed_edges = collapse_edge_with_fixed_vertices(nm, e, selection_points);
			for (NonManifoldEdge re : removed_edges)
			{
				EdgeQueueIt einfo = value<EdgeQueueIt>(nm, edge_queue_it, re);
				queue.erase(einfo);
			}
		}
		remove_attribute<NonManifoldEdge>(nm, edge_queue_it);
		nonmanifold_provider_->emit_connectivity_changed(nm);
	}
	
 	 HighsSolution point_selection_by_coverage_axis(SURFACE& surface, NONMANIFOLD& mv, double dilation_factor)
	{
		auto inner_position = get_attribute<Vec3, NonManifoldVertex>(mv, "position");
		auto sphere_radius = get_attribute<double, NonManifoldVertex>(mv, "sphere_radius");
		auto sample_position = get_attribute<Vec3, Vertex>(surface, "position");
		auto inner_point_nb = nb_cells<NonManifoldVertex>(mv);
		auto sample_point_nb = nb_cells<Vertex>(surface);
		Eigen::MatrixXd A(sample_point_nb, inner_point_nb);
		foreach_cell(mv, [&](NonManifoldVertex nv) {
			foreach_cell(surface, [&](Vertex v) {
				A(index_of(surface, v), index_of(mv, nv) ) =
					inside_sphere(value<Vec3>(surface, sample_position, v),
								  value<Vec3>(mv, inner_position, nv),
								  value<double>(mv, sphere_radius, nv) + dilation_factor) ? 1.0 : 0.0;
				return true;
			});
			return true;
		});
		HighsModel model;
		model.lp_.num_col_ = A.cols();
		model.lp_.num_row_ = A.rows();
		model.lp_.sense_ = ObjSense::kMinimize;
		// Adding decision variable bounds
		HighsVarType type = HighsVarType::kInteger;
		model.lp_.col_cost_ = std::vector<double>(model.lp_.num_col_, 1.0);
		model.lp_.col_lower_ = std::vector<double>(model.lp_.num_col_, 0.0);
		model.lp_.col_upper_ = std::vector<double>(model.lp_.num_col_, 1.0);
		model.lp_.row_lower_ = std::vector<double>(model.lp_.num_row_, 1.0);
		model.lp_.row_upper_ = std::vector<double>(model.lp_.num_row_, 1e30);
		model.lp_.integrality_ = std::vector<HighsVarType>(model.lp_.num_col_, type);
		model.lp_.a_matrix_.format_ = MatrixFormat::kColwise;
		model.lp_.a_matrix_.num_col_ = model.lp_.num_col_;
		model.lp_.a_matrix_.num_row_ = model.lp_.num_row_;
		model.lp_.a_matrix_.start_.clear();
		int currentStart = 0;
		for (int col = 0; col < model.lp_.a_matrix_.num_col_; ++col)
		{
			model.lp_.a_matrix_.start_.push_back(currentStart);

			for (int row = 0; row < model.lp_.a_matrix_.num_row_; ++row)
			{
				double value = A(row, col);
				if (value != 0.0)
				{
					model.lp_.a_matrix_.index_.push_back(row);
					model.lp_.a_matrix_.value_.push_back(value);
					currentStart++;
				}
			}
		}
		model.lp_.a_matrix_.start_.push_back(currentStart);

		Highs highs;
		HighsStatus status = highs.passModel(model);
		HighsSolution solution; 
		highs.setOptionValue("time_limit", 1000);
		if (status == HighsStatus::kOk)
		{
			highs.run();
			
			assert(status == HighsStatus::kOk);
			solution = highs.getSolution();
			

 			/*Eigen::VectorXd x = Eigen::VectorXd::Zero(A.cols());
			
 			for (int i = 0; i < solution.col_value.size(); ++i)
 			{
 				x[i] = solution.col_value[i];
 			}

			std::cout << " solution.row_value.size(): " << solution.row_value.size() << std::endl;
			for (int i = 0; i < solution.row_value.size(); ++i)
			{
				std::cout << solution.row_value[i] << " ";
			}
			std::cout << std::endl;
			std::cout << "compare " << std::endl;
			std::cout << A * x << std::endl;*/
			// Get the primal solution values
			
// 			auto ca_sphere_radius = get_attribute<double, NonManifoldVertex>(*ca, "sphere_radius");
// 			foreach_cell(*ca, [&](NonManifoldVertex v) { 
// 				value<double>(*ca, sphere_radius, v) += dilation_factor;
// 				return true;
// 			});

// 			auto ca_sphere_center = get_attribute<Vec3, NonManifoldVertex>(*ca, "position");
// 			uint32 outside_count = 0;
// 			foreach_cell(surface, [&](Vertex v) { 
// 				Vec3 pos = value<Vec3>(surface, sample_position, v);
// 				bool inside = false;
// 				foreach_cell(*ca, [&](NonManifoldVertex nv) {
// 					Vec3 capos = value<Vec3>(*ca, ca_sphere_center, nv);
// 					double radius = value<double>(*ca, ca_sphere_radius, nv);
// 					inside |= inside_sphere(pos, capos, radius);
// 					return true;
// 				});
// 				if (!inside)
// 				{
// 					outside_count++;
// 					std::cout << "Fault! " << outside_count << std::endl;
// 				}
// 					return true;
// 			});

			//construct_complete_power_diagram(ca_complet, power_point, point_info);
		}
		return solution;
	}
	
protected:
	void init() override
	{
		point_provider_ = static_cast<ui::MeshProvider<POINT>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<POINT>::name} + ")"));
		surface_provider_ = static_cast<ui::MeshProvider<SURFACE>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<SURFACE>::name} + ")"));
		nonmanifold_provider_ = static_cast<ui::MeshProvider<NONMANIFOLD>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<NONMANIFOLD>::name} + ")"));
	}

	void left_panel() override
	{
		imgui_mesh_selector(surface_provider_, selected_surface_mesh_, "Surface", [&](SURFACE& m) {
			selected_surface_mesh_ = &m;
			surface_provider_->mesh_data(m).outlined_until_ = App::frame_time_ + 1.0;
		});
		imgui_mesh_selector(nonmanifold_provider_, selected_medial_axis, "Medial_axis", [&](NONMANIFOLD& nm) {
			selected_medial_axis = &nm;
			nonmanifold_provider_->mesh_data(nm).outlined_until_ = App::frame_time_ + 1.0;
		});

		if (selected_surface_mesh_)
		{
			if (ImGui::Button("Power shape"))
			{
				compute_power_shape(*selected_surface_mesh_);
				
			}
			if (ImGui::Button("Original Medial Axis"))
				compute_original_power_diagram(*selected_surface_mesh_);
			if (selected_medial_axis)
			{
				static int32 vertices_to_remove = 1; 
				static float k = 0.00001f;
				ImGui::SliderInt("Vertices to delete", &vertices_to_remove, 1,
								 nb_cells<NonManifoldVertex>(*selected_medial_axis));
				ImGui::DragFloat("K", &k, 0.00001f, 0.0f, 1.0f, "%.5f"); 
				 if (ImGui::Button("QMAT"))
				{
					collapse_non_manifold_using_QMat(*selected_medial_axis,
																vertices_to_remove, k );
					
				}
				static float dilation_factor = 0.1f;
				ImGui::DragFloat("Dilation factor", &dilation_factor, 0.001f, 0.0f, 1.0f, "%.4f");
				if (ImGui::Button("Coverage Axis"))
				{
					solution = point_selection_by_coverage_axis(*selected_surface_mesh_, *selected_medial_axis, dilation_factor);
					
				}
				if (solution.col_value.size() > 0)
				{

					if (ImGui::Button("Collpase"))
						coverage_axis_collapse(*selected_medial_axis, solution.col_value);
					if (ImGui::Button("PD"))
						coverage_axis_PD(*selected_surface_mesh_, *selected_medial_axis, solution, dilation_factor);
				}
			}
				
				
			
		}
	}

private:
	POINT* surface_sample = nullptr;
	SURFACE* selected_surface_mesh_ = nullptr;
	NONMANIFOLD* selected_medial_axis = nullptr;
	std::shared_ptr<NonManifoldAttribute<double>> stability_ratio_ = nullptr;
	MeshProvider<POINT>* point_provider_;
	MeshProvider<SURFACE>* surface_provider_;
	MeshProvider<NONMANIFOLD>* nonmanifold_provider_;
	Regular medial_axis;
	HighsSolution solution;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_POWER_SHAPE_H_
