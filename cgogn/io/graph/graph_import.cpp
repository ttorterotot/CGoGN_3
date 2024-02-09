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

#include <cgogn/io/graph/graph_import.h>

#include <cgogn/core/types/incidence_graph/incidence_graph.h>
#include <cgogn/core/types/maps/cmap/graph.h>
#include <cgogn/core/types/animation/animation_skeleton.h>

#include <cgogn/core/types/cell_marker.h>

namespace cgogn
{

namespace io
{

void import_graph_data(Graph& g, const GraphImportData& graph_data)
{
	using Vertex = Graph::Vertex;

	auto vertex_dart = add_attribute<Dart, Vertex>(g, "__vertex_dart");

	for (uint32 vertex_id : graph_data.vertices_id_)
	{
		Vertex v = add_vertex(g, false);
		set_index(g, v, vertex_id);
		(*vertex_dart)[vertex_id] = v.dart_;
	}

	for (uint32 i = 0; i < uint32(graph_data.edges_vertex_indices_.size()); i += 2)
	{
		connect_vertices(g, Vertex((*vertex_dart)[graph_data.edges_vertex_indices_[i]]),
						 Vertex((*vertex_dart)[graph_data.edges_vertex_indices_[i + 1]]));
	}

	remove_attribute<Vertex>(g, vertex_dart);
}

void import_graph_data(IncidenceGraph& ig, const GraphImportData& graph_data)
{
	using Vertex = IncidenceGraph::Vertex;
	for (uint32 i = 0; i < uint32(graph_data.edges_vertex_indices_.size()); i += 2)
		add_edge(ig, Vertex(graph_data.edges_vertex_indices_[i]), Vertex(graph_data.edges_vertex_indices_[i + 1]));
}

void import_graph_data(AnimationSkeleton& as, const GraphImportData& graph_data)
{
	using P = std::pair<AnimationSkeleton::Joint, AnimationSkeleton::Joint>;
	std::vector<P> edges;
	edges.reserve(graph_data.edges_vertex_indices_.size() / 2);
	for (auto it = graph_data.edges_vertex_indices_.cbegin(); it != graph_data.edges_vertex_indices_.cend(); it += 2)
		edges.emplace_back(*it, *(it + 1));
	// Considering bones with lowest joint indices as higher in the hierarchy
	std::stable_sort(edges.begin(), edges.end(), [](const P& a, const P& b)
			{ return std::min(a.first, a.second) < std::min(b.first, b.second); });
	std::reverse(edges.begin(), edges.end()); // process back-to-front to reduce pop_back() average time complexity

	CellMarker<AnimationSkeleton, AnimationSkeleton::Joint> joint_added{as};

	while (!edges.empty())
	{
		AnimationSkeleton::Bone parent = INVALID_INDEX;
		bool first_marked = false, second_marked = false;

		auto it = edges.end();
		if (as.nb_bones() == 0)
			--it; // select last bone (lowest joint indices) as root
		else
		{
			// Find lowest-joint-index (last-in-vector) remaining bone
			// that connects to a joint that has already been marked
			while (--it != edges.begin())
			{
				first_marked = joint_added.is_marked(it->first);
				second_marked = joint_added.is_marked(it->second);

				cgogn_message_assert(!first_marked || !second_marked, "Duplicate edge");
				if (first_marked || second_marked)
					break;
			}
			cgogn_message_assert(it != edges.begin(), "More than one connected component");
		}

		// Add bone to the tree in the only direction that preserves the topology
		auto [first_joint, second_joint] = *it;
		if (first_marked) // base joint already in tree
			parent = get_parent_bone(as, first_joint);
		else if (second_marked) // tip (to-be base) joint already in tree
		{
			std::swap(first_joint, second_joint); // swap base and tip to preserve tree topology
			parent = get_parent_bone(as, first_joint);
		}
		else if (second_joint < first_joint)
			std::swap(first_joint, second_joint); // arbitrarily pick lowest index joint as base

		joint_added.mark(first_joint);
		joint_added.mark(second_joint);
		add_bone_from_existing_joints(as, parent, {first_joint, second_joint});
		edges.erase(it);
	}
}

} // namespace io

} // namespace cgogn
