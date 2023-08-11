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

#ifndef CGOGN_CORE_TYPES_MAPS_CMAP_CMAP2_H_
#define CGOGN_CORE_TYPES_MAPS_CMAP_CMAP2_H_

#include <cgogn/core/types/maps/cmap/cmap1.h>

namespace cgogn
{

struct CMap2 : public CMap1
{
	static const uint8 dimension = 2;

	using Parent = CMap1;

	using Vertex = Cell<PHI21>;
	using HalfEdge = Cell<DART>;
	using Edge = Cell<PHI2>;
	using Face = Cell<PHI1>;
	using Volume = Cell<PHI1_PHI2>;
	using CC = Volume;

	using Cells = std::tuple<Vertex, HalfEdge, Edge, Face, Volume>;

	std::shared_ptr<Attribute<Dart>> phi2_;

	CMap2() : CMap1()
	{
		phi2_ = add_relation("phi2");
	}
};

template <>
struct mesh_traits<CMap2>
{
	static constexpr const char* name = "CMap2";
	static constexpr const uint8 dimension = 2;

	using Parent = CMap2::Parent;

	using Vertex = CMap2::Vertex;
	using HalfEdge = CMap2::HalfEdge;
	using Edge = CMap2::Edge;
	using Face = CMap2::Face;
	using Volume = CMap2::Volume;

	using Cells = std::tuple<Vertex, HalfEdge, Edge, Face, Volume>;
	static constexpr const char* cell_names[] = {"Vertex", "HalfEdge", "Edge", "Face", "Volume"};

	template <typename T>
	using Attribute = CMapBase::Attribute<T>;
	using AttributeGen = CMapBase::AttributeGen;
	using MarkAttribute = CMapBase::MarkAttribute;
};

/*************************************************************************/
// Basic phi functions
/*************************************************************************/

inline Dart phi2(const CMap2& m, Dart d)
{
	return (*(m.phi2_))[d.index_];
}

inline void phi2_sew(CMap2& m, Dart d, Dart e)
{
	cgogn_assert(phi2(m, d) == d);
	cgogn_assert(phi2(m, e) == e);
	(*(m.phi2_))[d.index_] = e;
	(*(m.phi2_))[e.index_] = d;
}

inline void phi2_unsew(CMap2& m, Dart d)
{
	Dart e = phi2(m, d);
	(*(m.phi2_))[d.index_] = d;
	(*(m.phi2_))[e.index_] = e;
}

/*************************************************************************/
// Operators
/*************************************************************************/

CMap2::Vertex cut_edge(CMap2& m, CMap2::Edge e, bool set_indices = true);
CMap2::Vertex collapse_edge(CMap2& m, CMap2::Edge e, bool set_indices = true);
bool flip_edge(CMap2& m, CMap2::Edge e, bool set_indices = true);

bool edge_can_collapse(const CMap2& m, CMap2::Edge e);
bool edge_can_flip(const CMap2& m, CMap2::Edge e);

CMap2::Face add_face(CMap2& m, uint32 size, bool set_indices = true);
void merge_incident_faces(CMap2& m, CMap2::Edge e, bool set_indices = true);
CMap2::Edge cut_face(CMap2& m, CMap2::Vertex v1, CMap2::Vertex v2, bool set_indices = true);

CMap2::Volume add_pyramid(CMap2& m, uint32 size, bool set_indices = true);
CMap2::Volume add_prism(CMap2& m, uint32 size, bool set_indices = true);
void remove_volume(CMap2& m, CMap2::Volume v);

CMap2::Face close_hole(CMap2& m, Dart d, bool set_indices = true);
uint32 close(CMap2& m, bool set_indices = true);

void reverse_orientation(CMap2& m);

/*************************************************************************/
// Debugging helper functions
/*************************************************************************/

bool check_integrity(CMap2& m, bool verbose = true);

} // namespace cgogn

#endif // CGOGN_CORE_TYPES_MAPS_CMAP_CMAP2_H_