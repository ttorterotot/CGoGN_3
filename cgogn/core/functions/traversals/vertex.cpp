/*******************************************************************************
* CGoGN: Combinatorial and Geometric modeling with Generic N-dimensional Maps  *
* Copyright (C) 2015, IGG Group, ICube, University of Strasbourg, France       *
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

#include <cgogn/core/functions/traversals/vertex.h>

namespace cgogn
{

/*****************************************************************************/

// template <typename CELL, typename MESH>
// std::vector<typename mesh_traits<MESH>::Vertex> incident_vertices(const MESH& m, CELL c);

/*****************************************************************************/

///////////
// CMap1 //
///////////

std::vector<CMap1::Vertex> incident_vertices(const CMap1& m, CMap1::Face f)
{
	std::vector<CMap1::Vertex> vertices;
	m.foreach_dart_of_orbit(f, [&] (Dart d) -> bool { vertices.push_back(CMap1::Vertex(d)); return true; });
	return vertices;
}

///////////
// CMap2 //
///////////

std::vector<CMap2::Vertex> incident_vertices(const CMap2& m, CMap2::Edge e)
{
	std::vector<CMap2::Vertex> vertices;
	m.foreach_dart_of_orbit(e, [&] (Dart d) -> bool { vertices.push_back(CMap2::Vertex(d)); return true; });
	return vertices;
}

std::vector<CMap2::Vertex> incident_vertices(const CMap2& m, CMap2::Face f)
{
	std::vector<CMap2::Vertex> vertices;
	m.foreach_dart_of_orbit(f, [&] (Dart d) -> bool { vertices.push_back(CMap2::Vertex(d)); return true; });
	return vertices;
}

std::vector<CMap2::Vertex> incident_vertices(const CMap2& m, CMap2::Volume v)
{
	std::vector<CMap2::Vertex> vertices;
	foreach_incident_vertex(m, v, [&] (CMap2::Vertex v) -> bool { vertices.push_back(v); return true; });
	return vertices;
}

///////////
// CMap3 //
///////////

std::vector<CMap3::Vertex> incident_vertices(const CMap3& m, CMap3::Edge e)
{
	std::vector<CMap3::Vertex> vertices;
	static_cast<const CMap2&>(m).foreach_dart_of_orbit(CMap2::Edge(e.dart), [&] (Dart d) -> bool { vertices.push_back(CMap3::Vertex(d)); return true; });
	return vertices;
}

std::vector<CMap3::Vertex> incident_vertices(const CMap3& m, CMap3::Face f)
{
	std::vector<CMap3::Vertex> vertices;
	static_cast<const CMap2&>(m).foreach_dart_of_orbit(CMap2::Face(f.dart), [&] (Dart d) -> bool { vertices.push_back(CMap3::Vertex(d)); return true; });
	return vertices;
}

std::vector<CMap3::Vertex> incident_vertices(const CMap3& m, CMap3::Volume v)
{
	std::vector<CMap3::Vertex> vertices;
	DartMarkerStore marker(m);
	m.foreach_dart_of_orbit(v, [&] (Dart d) -> bool
	{
		if (!marker.is_marked(d))
		{
			// TODO: could mark only the darts of CMap2::Vertex(d)
			m.foreach_dart_of_orbit(CMap3::Vertex(d), [&] (Dart d) -> bool { marker.mark(d); return true; });
			vertices.push_back(CMap3::Vertex(d));
		}
		return true;
	});
	return vertices;
}

} // namespace cgogn
