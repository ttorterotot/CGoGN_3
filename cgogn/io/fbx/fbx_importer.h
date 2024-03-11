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
#include <cgogn/geometry/types/animation/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>
#include <cgogn/geometry/functions/bounding_box.h>
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
public:
	enum RotationOrder
	{
		Base = 8, // using octal for a clearer representation than binary while keeping size down
		XYZ = 0012, // Z -> Y -> X
		XZY = 0021, // Y -> Z -> X
		YXZ = 0102, // Z -> X -> Y
		YZX = 0120, // X -> Z -> Y
		ZXY = 0201, // Y -> X -> Z
		ZYX = 0210, // X -> Y -> Z
	};

protected:
	using Skeleton = AnimationSkeleton;
	using ObjectId = uint32;
	using AnimTimeT = double;
	using AnimScalar = Vec3::Scalar;
	using AnimationT = geometry::KeyframedAnimation<std::vector, AnimTimeT, AnimScalar>;

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

		std::array<std::optional<double>, 3> aod3_0_, aod3_1_, aod3_2_, aod3_3_;

	public:

		// Model

		inline constexpr std::array<std::optional<double>, 3>& pre_rotation(){ return aod3_0_; }
		inline constexpr std::array<std::optional<double>, 3>& post_rotation(){ return aod3_1_; }
		inline constexpr std::array<std::optional<double>, 3>& lcl_translation(){ return aod3_2_; }
		inline constexpr std::array<std::optional<double>, 3>& lcl_rotation(){ return aod3_3_; }

		inline constexpr const std::array<std::optional<double>, 3>& pre_rotation() const { return aod3_0_; }
		inline constexpr const std::array<std::optional<double>, 3>& post_rotation() const { return aod3_1_; }
		inline constexpr const std::array<std::optional<double>, 3>& lcl_translation() const { return aod3_2_; }
		inline constexpr const std::array<std::optional<double>, 3>& lcl_rotation() const { return aod3_3_; }

		// AnimationCurveNode

		inline constexpr std::array<std::optional<double>, 3>& d(){ return aod3_0_; }

		inline constexpr const std::array<std::optional<double>, 3>& d() const { return aod3_0_; }
	};

	struct Model
	{
		ObjectId id;
		ModelType type;
		std::string name;
		Properties properties;
		Skeleton::Bone bone;
		std::array<const AnimationT*, 6> animation;
	};

	struct Geometry
	{
		ObjectId id;
		std::string name;
		SurfaceImportData data;
	};

	struct AnimationCurve
	{
		ObjectId id;
		AnimScalar default_value;
		AnimationT animation;
	};

	struct AnimationCurveNode
	{
		ObjectId id;
		Properties properties;
	};

private:
	constexpr const static AnimTimeT ANIM_TIME_RATIO = 5e-11; // 0.05ns (eyeballed)

protected:

	inline FbxImporterBase(bool load_surfaces, bool load_skeletons)
			: load_surfaces_(load_surfaces), load_skeletons_(load_skeletons)
	{
	}

	/// @brief Opens and parses an FBX file
	/// @param path the file path to open
	void read(const std::string& path);

	ObjectId get_parent_id(const ObjectId& child_id);

	/// @brief Sets missing values for a property in `p` from `other_values`
	/// @param p the property to fill (existing values take priority over those from `other_values`)
	/// @param other_values the values to use
	/// @param property_name the name of the property
	/// @param warn_unequal whether or not to print a warning to stdout if the values are both present and different
	template <typename OtherContainer>
	void set_missing_values(Properties& p, const OtherContainer& other_values,
			const std::string& property_name, bool warn_unequal)
	{
		auto pi = PROPERTY_INFO_.find(property_name);
		if (pi == PROPERTY_INFO_.cend())
			return;

		const auto& [ptr, size] = pi->second(p);

		cgogn_assert(other_values.size() == size);

		for (size_t i = 0; i < size; ++i)
		{
			if (!ptr[i].has_value())
				ptr[i] = other_values[i];
			else if (warn_unequal && other_values[i].has_value() && other_values[i].value() != ptr[i].value())
				std::cout << "Warning: inconsistent values " << ptr[i].value() << " and " << other_values[i].value()
						<< " for property " << property_name;
		}
	}

private:
	void read_root(std::istream& is);
	void read_objects_node(std::istream& is);
	void read_objects_model_subnode(std::istream& is);
	void read_objects_geometry_subnode(std::istream& is);
	void read_objects_geometry_vertices_subnode(std::istream& is, SurfaceImportData& d);
	void read_objects_geometry_polygon_vertex_index_subnode(std::istream& is, SurfaceImportData& d);
	void read_objects_deformer_subnode(std::istream& is);
	void read_objects_animation_curve_subnode(std::istream& is);
	void read_objects_animation_curve_node_subnode(std::istream& is);
	void read_object_attributes(std::istream& is, ObjectId& id,
			std::string* name = nullptr, std::string* type = nullptr);
	void read_connections_node(std::istream& is);
	void read_fbx_header_extension_node(std::istream& is);
	void read_definitions_node(std::istream& is);
	void read_properties_70_subnode(std::istream& is, Properties& p);
	void read_property(std::istream& is, Properties& p);
	bool read_array(std::istream& is,
			std::function<void(uint32)> on_size, std::function<uint32()> read_values);
	void read_integer_and_warn_if_not_expected(std::istream& is, const std::string& nature, int expected_version);
	void skip_node(std::istream& is);
	void skip_value(std::istream& is);
	std::istream& skip_through_character(std::istream& is, char c);

	template <typename T, size_t Size>
	constexpr static std::pair<T*, size_t> std_array_g(std::array<T, Size>& arr){ return std::make_pair(arr.data(), Size); }

protected:
	std::vector<Model> models_;
	std::vector<Geometry> geometries_;
	std::vector<AnimationCurve> animation_curves_;
	std::vector<AnimationCurveNode> animation_curve_nodes_;
	std::vector<std::pair<ObjectId, ObjectId>> connections_oo_;
	std::vector<std::tuple<ObjectId, ObjectId, std::string>> connections_op_;

private:
	static inline const std::unordered_map<std::string,
			std::function<std::pair<std::optional<double>*, size_t>(Properties&)>> PROPERTY_INFO_ = {
		std::make_pair("PreRotation", [](Properties& p){ return std_array_g(p.pre_rotation()); }),
		std::make_pair("PostRotation", [](Properties& p){ return std_array_g(p.post_rotation()); }),
		std::make_pair("Lcl Translation", [](Properties& p){ return std_array_g(p.lcl_translation()); }),
		std::make_pair("Lcl Rotation", [](Properties& p){ return std_array_g(p.lcl_rotation()); }),
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

private:
	template <typename ContainerOfOptionals>
	geometry::Quaternion from_euler(const ContainerOfOptionals& xyz,
			const RotationOrder& rotation_order = RotationOrder::ZYX)
	{
		auto res = geometry::Quaternion::Identity();
		size_t ro = rotation_order;

		for (int i = 0; i < 3; ++i)
		{
			size_t axis = ro % RotationOrder::Base;
			ro /= RotationOrder::Base;

			if (xyz[axis])
				res = Eigen::AngleAxis<geometry::Quaternion::Scalar>(M_PI / 180.0 * xyz[axis].value(), AXES[axis])
						* res;
		}

		return res;
	}

	void load_surfaces(Map<std::string, Surface*>& surfaces, bool normalized)
	{
		using Vertex = typename mesh_traits<Surface>::Vertex;

		for (const Model& m : models_)
		{
			if (m.type != ModelType::Mesh)
				continue;

			Surface* surface = new Surface{};
			std::string name = m.name;
			decltype(surfaces.try_emplace(m.name, surface).first) it;
			bool inserted = false;

			while (std::tie(it, inserted) = surfaces.try_emplace(name, surface), !inserted)
				name += '_'; // disambiguate however we can

			for (const auto& c : connections_oo_)
			{
				ObjectId other_id;
				if (c.first == m.id)
					other_id = c.second;
				else if (c.second == m.id)
					other_id = c.first;
				else
					continue;

				const auto other_it = std::find_if(
						geometries_.begin(), geometries_.end(),
						[&](const Geometry& g){ return g.id == other_id; });

				if (other_it == geometries_.end())
					continue;

				import_surface_data(*surface, other_it->data);

				if (normalized)
					if (auto attr = get_attribute<Vec3, Vertex>(*surface,
							other_it->data.vertex_position_attribute_name_))
						geometry::rescale(*attr, 1);
			}

			// TODO connections_op_
		}
	}

	void load_skeletons(Map<std::string, Skeleton*>& skeletons)
	{
		if (std::find_if(models_.cbegin(), models_.cend(),
				[](const Model& m) { return m.type == ModelType::LimbNode; }) == models_.cend())
			return;

		Skeleton* skeleton = new Skeleton{};

		for (Model& m : models_)
			if (m.type == ModelType::LimbNode)
				m.bone = add_root(*skeleton);

		for (const Model& m : models_)
		{
			if (m.type != ModelType::LimbNode)
				continue;

			const ObjectId& parent_id = get_parent_id(m.id);

			if (parent_id == INVALID_INDEX)
				continue;

			const auto parent_model_it = std::find_if(models_.cbegin(), models_.cend(),
					[&](const Model& parent) { return parent.id == parent_id; });

			if (parent_model_it != models_.cend())
			{
				cgogn_assert(parent_model_it->type == ModelType::LimbNode);
				attach_bone(*skeleton, m.bone, parent_model_it->bone);
			}
		}

		// Handle bone animations
		// TODO: handling multiple animations in the same file (layers?)

		for (const auto& c : connections_op_)
		{
			// Get bone's animation curve node and targeted property

			const auto& [curve_node_id, model_id, property_name] = c;

			const auto model_it = std::find_if(models_.begin(), models_.end(),
					[&](const Model& model){ return model.id == model_id; });

			if (model_it == models_.end())
				continue;

			const auto curve_node_it = std::find_if(animation_curve_nodes_.begin(), animation_curve_nodes_.end(),
					[&](const AnimationCurveNode& node){ return node.id == curve_node_id; });

			if (curve_node_it == animation_curve_nodes_.end())
				continue;

			for (const auto& c_ : connections_op_)
			{
				// Get animation curve node's animation curves and targeted property

				const auto& [curve_id, curve_node_id_, property_name_] = c_;

				if (curve_node_id != curve_node_id_)
					continue;

				const auto curve_it = std::find_if(animation_curves_.cbegin(), animation_curves_.cend(),
						[&](const AnimationCurve& curve){ return curve.id == curve_id; });

				if (curve_it == animation_curves_.cend())
					continue;

				// Compare node value with curve default (*)
				set_missing_values(curve_node_it->properties, std::array<std::optional<double>, 1>{curve_it->default_value},
						property_name_, false);

				// Get animation

				size_t anim_id = 6; // 6 indicates ignored

				// Get transformation type offset
				if (property_name == "Lcl Translation"s)
					anim_id = 0;
				else if (property_name == "Lcl Rotation"s)
					anim_id = 3;

				// Get axis offset
				if (property_name_ == "d|Z"s)
					anim_id += 2;
				else if (property_name_ == "d|Y"s)
					++anim_id;
				else if (property_name_ != "d|X"s)
					anim_id = 6; // not d|<axis>, ignore

				if (anim_id != 6)
					model_it->animation[anim_id] = &curve_it->animation;
			}

			// Compare model value with node's (*)
			set_missing_values(model_it->properties, curve_node_it->properties.d(), property_name, true);

			// (*) should be equal if both are present, if the former isn't, then try to get a value from the latter
		}

		using RT = geometry::RigidTransformation<geometry::Quaternion, Vec3>;
		using KA_RT = geometry::KeyframedAnimation<std::vector, AnimTimeT, RT>;
		using KA_DQ = geometry::KeyframedAnimation<std::vector, AnimTimeT, geometry::DualQuaternion>;

		add_attribute<Vec3, Skeleton::Joint>(*skeleton, "position");
		// TODO improve names (e.g. using file name) and handle collisions
		// (expecting segfault on second import from returned nullptr due to name already being taken)
		auto& attr_anim_rt = *add_attribute<KA_RT, Skeleton::Bone>(*skeleton, "TODO RT placeholder name");
		auto& attr_anim_rt_b = *add_attribute<KA_RT, Skeleton::Bone>(*skeleton, "TODO RT binding placeholder name");
		auto& attr_anim_dq = *add_attribute<KA_DQ, Skeleton::Bone>(*skeleton, "TODO DQ placeholder name");
		auto& attr_anim_dq_b = *add_attribute<KA_DQ, Skeleton::Bone>(*skeleton, "TODO DQ binding placeholder name");

		for (const Model& m : models_)
		{
			if (m.type != ModelType::LimbNode)
				continue;

			KA_RT anim_rt; // RigidTransformation animation
			KA_RT anim_rt_b; // RigidTransformation bind pose
			KA_DQ anim_dq; // DualQuaternion animation
			KA_DQ anim_dq_b; // DualQuaternion bind pose

			// Get keyframe times
			std::set<AnimTimeT> times;
			{
				std::vector<AnimationT> anim_copies;
				for (const auto& anim : m.animation)
					if (anim)
						anim_copies.push_back(*anim);
				times = AnimationT::get_unique_keyframe_times(anim_copies);
			}

			const auto& rotation_order = RotationOrder::XYZ;

			const auto lcl_translation_value_d = [&](const size_t& index) {
					return m.properties.lcl_translation()[index].value_or(0.0);
			};

			auto pre_rotation = from_euler(m.properties.pre_rotation());
			auto post_rotation = from_euler(m.properties.post_rotation());
			auto lcl_rotation = from_euler(m.properties.lcl_rotation());
			auto total_lcl_rotation = pre_rotation * lcl_rotation * post_rotation;
			Vec3 lcl_translation{lcl_translation_value_d(0), lcl_translation_value_d(1), lcl_translation_value_d(2)};

			anim_rt_b.emplace_back(0.0, RT{total_lcl_rotation, lcl_translation});
			anim_dq_b.emplace_back(0.0, geometry::DualQuaternion::from_tr(lcl_translation, total_lcl_rotation));

			// We assume the interpolation to be linear, so we can create full transform keyframes by interpolating
			// each component along its own animation, instead of passing the split components beyond
			for (const auto& time : times)
			{
				const auto has_component = [&](const size_t& offset){
						return m.animation[offset] || m.animation[offset + 1] || m.animation[offset + 2]; };
				const auto anim_value = [&](const size_t& index) -> std::optional<AnimScalar> {
						return m.animation[index] ?
								m.animation[index]->get_value(time) : std::optional<AnimScalar>(); };

				auto t = has_component(0) ?
						Vec3{anim_value(0).value_or(0.0), anim_value(1).value_or(0.0), anim_value(2).value_or(0.0)} :
						lcl_translation;
				auto r = has_component(3) ?
						geometry::Quaternion{pre_rotation
								* from_euler(std::array<std::optional<AnimScalar>, 3>
										{anim_value(3), anim_value(4), anim_value(5)})
								* post_rotation} :
						total_lcl_rotation;

				anim_rt.emplace_back(time, RT{r, t});
				anim_dq.emplace_back(time, geometry::DualQuaternion::from_tr(t, r));
			}

			if (times.empty())
			{
				anim_rt = anim_rt_b;
				anim_dq = anim_dq_b;
			}

			attr_anim_rt[m.bone] = anim_rt;
			attr_anim_rt_b[m.bone] = anim_rt_b;
			attr_anim_dq[m.bone] = anim_dq;
			attr_anim_dq_b[m.bone] = anim_dq_b;
		}

		std::string name;
		bool inserted = false;
		while (std::tie(std::ignore, inserted) = skeletons.try_emplace(name, skeleton), !inserted)
			name += '_'; // disambiguate however we can
	}

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

		if (load_surfaces)
			importer.load_surfaces(surfaces, normalized);

		if (load_skeletons)
			importer.load_skeletons(skeletons);
	}

private:
	static inline const std::array<Vec3, 3> AXES = {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}};
};

} // namespace io

} // namespace cgogn

#endif // CGOGN_IO_FBX_IMPORTER_H_
