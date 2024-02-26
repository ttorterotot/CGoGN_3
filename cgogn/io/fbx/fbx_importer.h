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

#ifndef CGOGN_IO_FBX_IMPORTER_H_
#define CGOGN_IO_FBX_IMPORTER_H_

#include <string>
#include <fstream>
#include <unordered_map>

#include <cgogn/io/cgogn_io_export.h>

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/io/surface/surface_import.h>

namespace cgogn
{

struct CMap2;
struct GMap2;
struct IncidenceGraph;
struct TriangleSoup;

namespace io
{

class FbxImporterBase
{
protected:
	using Skeleton = AnimationSkeleton;

protected:

	inline FbxImporterBase(bool load_surfaces, bool load_skeletons)
			: load_surfaces_(load_surfaces), load_skeletons_(load_skeletons)
	{
	}

	void read(const std::string& path);
	void read_root(std::ifstream& is);
	void read_objects_node(std::ifstream& is);
	void read_connections_node(std::ifstream& is);
	void skip_node(std::ifstream& is);

private:
	bool load_surfaces_;
	bool load_skeletons_;
};

template <typename Surface>
class FbxImporter : public FbxImporterBase
{
public:
	template <typename T, typename U>
	using Map = typename std::unordered_map<T, U>;

private:
	using FbxImporterBase::FbxImporterBase; // inherit constructor

public:
	FbxImporter() = delete;

	static void load(const std::string& path,
			Map<std::string, Surface*>& surfaces,
			Map<std::string, Skeleton*>& skeletons,
			bool load_surfaces = true, bool load_skeletons = true, bool normalized = false)
	{
		FbxImporter importer{load_surfaces, load_skeletons};
		importer.read(path);
	}
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FBX_IMPORTER_H_
