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
#include <optional>
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
	using ObjectId = uint32;

	enum class ModelType
	{
		None,
		Mesh,
		LimbNode,
	};

	class Properties
	{
	private:
		// Shared fields to save memory for non-overlapping usages
		// (usage of unions for this purpose is not ISO C++)

		std::array<std::optional<double>, 3> aod3_0_, aod3_1_;

	public:

		// Model

		inline constexpr std::array<std::optional<double>, 3>& pre_rotation(){ return aod3_0_; }
		inline constexpr std::array<std::optional<double>, 3>& lcl_translation(){ return aod3_1_; }

		// AnimationCurveNode

		inline constexpr std::array<std::optional<double>, 3>& d(){ return aod3_0_; }
	};

	struct Model
	{
		ObjectId id;
		ModelType type;
		std::string name;
		Properties properties;
	};

protected:

	inline FbxImporterBase(bool load_surfaces, bool load_skeletons)
			: load_surfaces_(load_surfaces), load_skeletons_(load_skeletons)
	{
	}

	/// @brief Opens and parses an FBX file
	/// @param path the file path to open
	void read(const std::string& path);

private:
	void read_root(std::istream& is);
	void read_objects_node(std::istream& is);
	void read_objects_model_subnode(std::istream& is);
	void read_objects_geometry_subnode(std::istream& is);
	void read_connections_node(std::istream& is);
	void read_fbx_header_extension_node(std::istream& is);
	void read_definitions_node(std::istream& is);
	void read_properties_70_subnode(std::istream& is, Properties& p);
	void read_property(std::istream& is, Properties& p);
	void read_integer_and_warn_if_not_expected(std::istream& is, const std::string& nature, int expected_version);
	void skip_node(std::istream& is);
	void skip_value(std::istream& is);
	std::istream& skip_through_character(std::istream& is, char c);

	template <typename T, size_t Size>
	constexpr static std::pair<T*, size_t> std_array_g(std::array<T, Size>& arr){ return std::make_pair(arr.data(), Size); }

protected:
	std::vector<Model> models_;

private:
	static inline const std::unordered_map<std::string,
			std::function<std::pair<std::optional<double>*, size_t>(Properties&)>> PROPERTY_INFO_ = {
		std::make_pair("PreRotation", [](Properties& p){ return std_array_g(p.pre_rotation()); }),
		std::make_pair("Lcl Translation", [](Properties& p){ return std_array_g(p.lcl_translation()); }),
		std::make_pair("d|X", [](Properties& p){ return std::make_pair(p.d().data(), 1); }),
		std::make_pair("d|Y", [](Properties& p){ return std::make_pair(p.d().data() + 1, 1); }),
		std::make_pair("d|Z", [](Properties& p){ return std::make_pair(p.d().data() + 2, 1); }),
	};

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

	/// @brief Opens and parses an FBX file, filling the maps with objects inside
	/// @param path the file path to open
	/// @param surfaces a map to fill with surfaces described in the file
	/// @param skeletons a map to fill with animation skeletons described in the file
	/// @param load_surfaces whether or not to actually read surfaces
	/// @param load_skeletons whether or not to actually read skeletons
	/// @param normalized whether or not to normalize the positions for each surface (can offset it from bones)
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
