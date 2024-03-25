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

#ifndef CGOGN_CORE_FUNCTIONS_MESH_INFO_H_
#define CGOGN_CORE_FUNCTIONS_MESH_INFO_H_

#include <cgogn/core/functions/traversals/vertex.h>
#include <cgogn/core/utils/numerics.h>
#include <cgogn/core/utils/tuples.h>

namespace cgogn
{

template <typename MESH>
struct mesh_traits;

// compile time check if a mesh has a given cell type

template <typename T>
struct void_
{
	typedef void type;
};

template <typename MESH, typename = void>
struct has_vertex : std::false_type
{
};
template <typename MESH>
struct has_vertex<MESH, typename void_<typename MESH::Vertex>::type> : std::true_type
{
};
template <typename MESH>
inline constexpr bool has_vertex_v = has_vertex<MESH>::value;

template <typename MESH, typename = void>
struct has_edge : std::false_type
{
};
template <typename MESH>
struct has_edge<MESH, typename void_<typename MESH::Edge>::type> : std::true_type
{
};
template <typename MESH>
inline constexpr bool has_edge_v = has_edge<MESH>::value;

template <typename MESH, typename = void>
struct has_face : std::false_type
{
};
template <typename MESH>
struct has_face<MESH, typename void_<typename MESH::Face>::type> : std::true_type
{
};
template <typename MESH>
inline constexpr bool has_face_v = has_face<MESH>::value;

template <typename MESH, typename = void>
struct has_volume : std::false_type
{
};
template <typename MESH>
struct has_volume<MESH, typename void_<typename MESH::Volume>::type> : std::true_type
{
};
template <typename MESH>
inline constexpr bool has_volume_v = has_volume<MESH>::value;

template <typename MESH, typename CELL>
struct has_cell_type
{
	static constexpr bool value = is_in_tuple_v<CELL, typename mesh_traits<MESH>::Cells>;
};

template <typename MESH, typename CELL>
inline constexpr bool has_cell_type_v = has_cell_type<MESH, CELL>::value;

// some generic functions to get info about a mesh and its cells

template <typename CELL, typename MESH>
std::string cell_name(const MESH&)
{
	static_assert(is_in_tuple_v<CELL, typename mesh_traits<MESH>::Cells>, "CELL not supported in this MESH");
	return mesh_traits<MESH>::cell_names[tuple_type_index<CELL, typename mesh_traits<MESH>::Cells>::value];
}

template <typename CELL, typename MESH>
uint32 nb_cells(const MESH& m)
{
	static_assert(is_in_tuple_v<CELL, typename mesh_traits<MESH>::Cells>, "CELL not supported in this MESH");
	uint32 result = 0;
	foreach_cell(m, [&](CELL) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
bool is_simplicial(const MESH& m)
{
	bool res = true;
	if constexpr (mesh_traits<MESH>::dimension == 2)
	{
		foreach_cell(m, [&](typename mesh_traits<MESH>::Face f) -> bool {
			auto vertices = first_incident_vertices<4>(m, f);
			res = vertices[2].is_valid() && !vertices[3].is_valid();
			//    ^-- equivalent to incident_vertices(m, f).size() == 3
			return res;
		});
	}
	if constexpr (mesh_traits<MESH>::dimension == 3)
	{
		foreach_cell(m, [&](typename mesh_traits<MESH>::Volume v) -> bool {
			auto vertices = first_incident_vertices<5>(m, v);
			res = vertices[3].is_valid() && !vertices[4].is_valid();
			//    ^-- equivalent to incident_vertices(m, v).size() == 4
			return res;
		});
	}
	return res;
}

template <typename MESH>
uint32 degree(const MESH& m, typename mesh_traits<MESH>::Vertex v)
{
	uint32 result = 0;
	foreach_incident_edge(m, v, [&](typename mesh_traits<MESH>::Edge) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
uint32 degree(const MESH& m, typename mesh_traits<MESH>::Edge e)
{
	uint32 result = 0;
	foreach_incident_face(m, e, [&](typename mesh_traits<MESH>::Face) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
uint32 degree(const MESH& m, typename mesh_traits<MESH>::Face f)
{
	uint32 result = 0;
	foreach_incident_volume(m, f, [&](typename mesh_traits<MESH>::Volume) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
uint32 codegree(const MESH& m, typename mesh_traits<MESH>::Edge e)
{
	uint32 result = 0;
	foreach_incident_vertex(m, e, [&](typename mesh_traits<MESH>::Vertex) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
uint32 codegree(const MESH& m, typename mesh_traits<MESH>::Face f)
{
	uint32 result = 0;
	foreach_incident_edge(m, f, [&](typename mesh_traits<MESH>::Edge) -> bool {
		++result;
		return true;
	});
	return result;
}

template <typename MESH>
uint32 codegree(const MESH& m, typename mesh_traits<MESH>::Volume v)
{
	uint32 result = 0;
	foreach_incident_face(m, v, [&](typename mesh_traits<MESH>::Face) -> bool {
		++result;
		return true;
	});
	return result;
}

} // namespace cgogn

#endif // CGOGN_CORE_FUNCTIONS_MESH_INFO_H_
