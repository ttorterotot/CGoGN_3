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

#ifndef CGOGN_GEOMETRY_ALGOS_DISTANCE_H_
#define CGOGN_GEOMETRY_ALGOS_DISTANCE_H_

#include <utility>

#include <cgogn/core/types/cells_set.h>

#include <cgogn/geometry/algos/area.h>
#include <cgogn/geometry/algos/gradient.h>
#include <cgogn/geometry/algos/laplacian.h>

#include <cgogn/geometry/functions/distance.h>

#include <cgogn/geometry/types/vector_traits.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace cgogn
{

namespace geometry
{

template <typename MESH>
Vec3 closest_point_on_surface(const MESH& m,
							  const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							  const Vec3& p)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	using Face = typename mesh_traits<MESH>::Face;

	Vec3 closest(0, 0, 0);
	Scalar min_dist = std::numeric_limits<Scalar>::max();

	foreach_cell(m, [&](Face f) -> bool {
		auto vertices = first_incident_vertices<3>(m, f);
		// std::vector<const Vec3*> vertices_position;
		// std::transform(vertices.begin(), vertices.end(), std::back_inserter(vertices_position),
		// 			   [&](Vertex v) -> const Vec3* { return &value<Vec3>(m, vertex_position, v); });
		Scalar u, v, w;
		// assume triangle faces
		const Vec3& a = value<Vec3>(m, vertex_position, vertices[0]);
		const Vec3& b = value<Vec3>(m, vertex_position, vertices[1]);
		const Vec3& c = value<Vec3>(m, vertex_position, vertices[2]);
		closest_point_in_triangle(p, a, b, c, u, v, w);
		Vec3 pos = u * a + v * b + w * c;
		Scalar dist = (pos - p).squaredNorm();
		if (dist < min_dist)
		{
			closest = pos;
			min_dist = dist;
		}
		return true;
	});

	return closest;
}

template <typename MESH, typename GRID>
Vec3 closest_point_on_surface(const MESH& m,
							  const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							  const GRID& g, const Vec3& p)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	using Face = typename mesh_traits<MESH>::Face;

	Vec3 closest(0, 0, 0);
	Scalar min_dist = std::numeric_limits<Scalar>::max();

	g.foreach_face_around(p, [&](Face f) {
		auto vertices = first_incident_vertices<3>(m, f);
		// std::vector<const Vec3*> vertices_position;
		// std::transform(vertices.begin(), vertices.end(), std::back_inserter(vertices_position),
		// 			   [&](Vertex v) -> const Vec3* { return &value<Vec3>(m, vertex_position, v); });
		Scalar u, v, w;
		// assume triangle faces
		const Vec3& a = value<Vec3>(m, vertex_position, vertices[0]);
		const Vec3& b = value<Vec3>(m, vertex_position, vertices[1]);
		const Vec3& c = value<Vec3>(m, vertex_position, vertices[2]);
		closest_point_in_triangle(p, a, b, c, u, v, w);
		Vec3 pos = u * a + v * b + w * c;
		Scalar dist = (pos - p).squaredNorm();
		if (dist < min_dist)
		{
			closest = pos;
			min_dist = dist;
		}
	});

	return closest;
}

template <typename MESH>
inline Vec3 closest_point_on_edge(const MESH& m,
								   const typename mesh_traits<MESH>::Edge& e,
								   const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
								   const Vec3& p)
{
	auto vertices = first_incident_vertices<2>(m, e);
	const Vec3& p0 = value<Vec3>(m, vertex_position, vertices[0]);
	const Vec3& p1 = value<Vec3>(m, vertex_position, vertices[1]);
	const Vec3 u = p - p0;
	const Vec3 v = p1 - p0;

	const auto vv = v.squaredNorm();
	if (vv == 0.0) // zero-length edge, pick one of the points (which are equal +- floating point precision)
		return p0;

	const auto t = std::clamp(u.dot(v) / vv, 0.0, 1.0);
	return (1.0 - t) * p0 + t * p1;
	// We could also return p0 + t * v, but it wouldn't ensure we get exactly p1 for t := 1; (1 - t) * p0 + t * p1 does
	// (we consider floating point inaccuracies to be OK on the strict interval, but may not prefer any at extremities)
}

template <typename MESH>
std::pair<typename mesh_traits<MESH>::Edge, Vec3> closest_edge_and_point_on_edges(const MESH& m,
							const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							const Vec3& p)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	using Edge = typename mesh_traits<MESH>::Edge;

	Edge closest_edge = INVALID_INDEX;
	Vec3 closest_pos = Vec3::Zero();
	Scalar min_dist = std::numeric_limits<Scalar>::max();

	foreach_cell(m, [&](Edge e) -> bool {
		const Vec3 pos = closest_point_on_edge(m, e, vertex_position, p);
		const Scalar dist = (pos - p).squaredNorm();
		if (dist < min_dist)
		{
			closest_edge = e;
			closest_pos = pos;
			min_dist = dist;
		}
		return true;
	});

	return std::make_pair(closest_edge, closest_pos);
}

template <typename MESH>
typename mesh_traits<MESH>::Edge closest_edge(const MESH& m,
							const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							const Vec3& p)
{
	return closest_edge_and_point_on_edges(m, vertex_position, p).first;
}

template <typename MESH>
Vec3 closest_point_on_edges(const MESH& m,
							const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							const Vec3& p)
{
	return closest_edge_and_point_on_edges(m, vertex_position, p).second;
}

template <typename MESH>
void compute_geodesic_distance(MESH& m, const typename mesh_traits<MESH>::template Attribute<Vec3>* vertex_position,
							   const CellsSet<MESH, typename mesh_traits<MESH>::Vertex>* source_vertices,
							   typename mesh_traits<MESH>::template Attribute<Scalar>* vertex_geodesic_distance)
{
	using Vertex = typename mesh_traits<MESH>::Vertex;
	using Face = typename mesh_traits<MESH>::Face;

	auto vertex_index = add_attribute<uint32, Vertex>(m, "__vertex_index");

	uint32 nb_vertices = 0;
	foreach_cell(m, [&](Vertex v) -> bool {
		value<uint32>(m, vertex_index, v) = nb_vertices++;
		return true;
	});

	Eigen::SparseMatrix<Scalar, Eigen::ColMajor> Lc =
		geometry::cotan_operator_matrix(m, vertex_index.get(), vertex_position);

	auto vertex_area = add_attribute<Scalar, Vertex>(m, "__vertex_area");
	geometry::compute_area<Vertex>(m, vertex_position, vertex_area.get());

	Eigen::VectorXd A(nb_vertices);
	parallel_foreach_cell(m, [&](Vertex v) -> bool {
		uint32 vidx = value<uint32>(m, vertex_index, v);
		A(vidx) = value<Scalar>(m, vertex_area, v);
		return true;
	});

	Eigen::VectorXd u0(nb_vertices);
	u0.setZero();
	source_vertices->foreach_cell([&](Vertex v) {
		uint32 vidx = value<uint32>(m, vertex_index, v);
		u0(vidx) = 1.0;
	});

	Scalar h = geometry::mean_edge_length(m, vertex_position);
	Scalar t = h * h;

	Eigen::SparseMatrix<Scalar, Eigen::ColMajor> Am(A.asDiagonal());
	Eigen::SimplicialLDLT<Eigen::SparseMatrix<Scalar, Eigen::ColMajor>> heat_solver(Am - t * Lc);
	Eigen::VectorXd u = heat_solver.solve(u0);

	auto vertex_heat = get_or_add_attribute<Scalar, Vertex>(m, "__vertex_heat");
	parallel_foreach_cell(m, [&](Vertex v) -> bool {
		uint32 vidx = value<uint32>(m, vertex_index, v);
		value<Scalar>(m, vertex_heat, v) = u(vidx);
		return true;
	});

	auto face_heat_gradient = get_or_add_attribute<Vec3, Face>(m, "__face_heat_gradient");
	geometry::compute_gradient_of_vertex_scalar_field(m, vertex_position, vertex_heat.get(), face_heat_gradient.get());

	auto vertex_heat_gradient_div = get_or_add_attribute<Scalar, Vertex>(m, "__vertex_heat_gradient_div");
	geometry::compute_div_of_face_vector_field(m, vertex_position, face_heat_gradient.get(),
											   vertex_heat_gradient_div.get());

	Eigen::VectorXd b(nb_vertices);
	parallel_foreach_cell(m, [&](Vertex v) -> bool {
		uint32 vidx = value<uint32>(m, vertex_index, v);
		b(vidx) = value<Scalar>(m, vertex_heat_gradient_div, v);
		return true;
	});

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<Scalar, Eigen::ColMajor>> poisson_solver(Lc);
	Eigen::VectorXd dist = poisson_solver.solve(b);

	Scalar min = dist.minCoeff();
	parallel_foreach_cell(m, [&](Vertex v) -> bool {
		uint32 vidx = value<uint32>(m, vertex_index, v);
		value<Scalar>(m, vertex_geodesic_distance, v) = dist(vidx) - min;
		return true;
	});

	remove_attribute<Vertex>(m, vertex_index);
	remove_attribute<Vertex>(m, vertex_area);
	// remove_attribute<Vertex>(m, vertex_heat);
	// remove_attribute<Face>(m, face_heat_gradient);
	// remove_attribute<Vertex>(m, vertex_heat_gradient_div);
}

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_ALGOS_DISTANCE_H_
