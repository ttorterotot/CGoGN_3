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

#include <variant>

#include <cgogn/io/utils.h>
#include <cgogn/io/fbx/fbx_importer.h>

using namespace std::literals::string_literals;

namespace cgogn
{

namespace io
{

void FbxImporterBase::read(const std::string& path)
{
	Scoped_C_Locale loc;

	std::ifstream is(path.c_str(), std::ios::in);
	std::istream::sentry se(is, true); // clean input stream

	read_root(is);
}

// Reads an FBX input from the beginning (node level)
void FbxImporterBase::read_root(std::istream& is)
{
	std::string node_key;
	char c;

	node_key.reserve(32);

	while (is.get(c))
	{
		if (std::isspace(c)) // ignore whitespace
			continue;

		if (c == ':') // end of key
		{
			if (node_key == "Objects"s)
				read_objects_node(is);
			else if (node_key == "Connections"s)
				read_connections_node(is);
			else if (node_key == "FBXHeaderExtension"s)
				read_fbx_header_extension_node(is);
			else if (node_key == "Definitions"s)
				read_definitions_node(is);
			else
				skip_node(is);

			node_key.clear();
		}
		else if (c == ';' && node_key.empty()) // semicolon on start of line is comment
			skip_through_character(is, '\n');
		else // read key
			node_key += c;
	}
}

// Reads an Objects node from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_node(std::istream& is)
{
	std::string subnode_key;
	char c;

	subnode_key.reserve(32);

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return;
		case ':':
			if (subnode_key == "Model"s)
				read_objects_model_subnode(is);
			else if (subnode_key == "Geometry"s)
				read_objects_geometry_subnode(is);
			else if (subnode_key == "Deformer"s)
				read_objects_deformer_subnode(is);
			else if (subnode_key == "AnimationCurve"s)
				read_objects_animation_curve_subnode(is);
			else if (subnode_key == "AnimationCurveNode"s)
				read_objects_animation_curve_node_subnode(is);
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid syntax for Objects node" << std::endl;
}

// Reads a Model subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_model_subnode(std::istream& is)
{
	std::variant<Model, MeshModel, LimbNodeModel> model_variant{}; // default-constructed to the Model type
	auto* model = std::get_if<Model>(&model_variant); // necessarily non-null because we know that variant is assigned
	std::string type;
	std::string subnode_key;
	char c;

	read_object_attributes(is, model->id, &model->name, &type);

	if (type == "Mesh"s)
	{
		if (!load_surfaces_)
		{
			skip_node(is);
			return;
		}
		model_variant = MeshModel{*model};
		model = std::get_if<MeshModel>(&model_variant);
	}
	else if (type == "LimbNode"s)
	{
		if (!load_skeleton_)
		{
			skip_node(is);
			return;
		}
		model_variant = LimbNodeModel{*model};
		model = std::get_if<LimbNodeModel>(&model_variant);
	}

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			if (auto* m = std::get_if<LimbNodeModel>(&model_variant))
				models_limb_node_.push_back(std::move(*m));
			else if (auto* m = std::get_if<MeshModel>(&model_variant))
				models_mesh_.push_back(std::move(*m));
			// Other model types are ignored because we don't have a use for them yet
			// It would simply be a matter of having a vector of plain models to push to
			return;
		case ':':
			if (subnode_key == "Version"s)
				read_integer_and_warn_if_not_expected(is, "model version", 232);
			else if (subnode_key == "Properties70"s)
				read_properties_70_subnode(is, model->properties);
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid syntax for model " << model->name << std::endl;
}

// Reads a Geometry subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_subnode(std::istream& is)
{
	Geometry geometry{};
	std::string type;
	std::string subnode_key;
	char c;

	subnode_key.reserve(32);

	read_object_attributes(is, geometry.id, &geometry.name, &type);

	if (type != "Mesh"s)
		std::cout << "Warning: expected geometry of type Mesh for geometry " << geometry.name
				<< " but got " << type << std::endl;

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			geometries_.push_back(std::move(geometry));
			return;
		case ':':
			if (subnode_key == "Vertices"s)
				read_objects_geometry_vertices_subnode(is, geometry.data);
			else if (subnode_key == "PolygonVertexIndex"s)
				read_objects_geometry_polygon_vertex_index_subnode(is, geometry.data);
			else if (subnode_key == "GeometryVersion"s)
				read_integer_and_warn_if_not_expected(is, "geometry version", 124);
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid syntax for geometry " << geometry.name << std::endl;
}

// Reads a Vertices subnode (inside a Geometry subnode in an Objects node)
// from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_vertices_subnode(std::istream& is, SurfaceImportData& d)
{
	Vec3 v = Vec3::Zero();
	int coef = 0;

	if (read_array(is,
			[&](uint32 size){
				d.nb_vertices_ = size / 3; // number in the file is number of coefficients
				d.vertex_position_.reserve(d.nb_vertices_);
			},
			[&]{
				is >> v[coef];
				if (++coef == 3)
				{
					d.vertex_position_.push_back(v);
					v = Vec3::Zero();
					coef = 0;
				}
				return 1;
			}))
	{
		cgogn_assert(d.nb_vertices_ == d.vertex_position_.size());
	}
	else
		std::cout << "Warning: invalid syntax for Vertices subnode" << std::endl;
}

// Reads a PolygonVertexIndex subnode (inside a Geometry subnode in an Objects node)
// from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_polygon_vertex_index_subnode(std::istream& is, SurfaceImportData& d)
{
	int nb_vertices = 0;

	if (!read_array(is,
			[&](uint32 size){ d.faces_vertex_indices_.reserve(size); },
			[&]{
				int32 id;
				is >> id;
				if (id < 0)
				{
					id = -1 - id;
					++d.nb_faces_;
					d.faces_nb_vertices_.push_back(nb_vertices + 1);
					nb_vertices = 0;
				}
				else
					++nb_vertices;
				d.faces_vertex_indices_.push_back(id);
				return 1;
			}))
		std::cout << "Warning: invalid syntax for PolygonVertexIndex subnode" << std::endl;
}

// Reads a Deformer subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_deformer_subnode(std::istream& is)
{
	skip_node(is); // TODO
}

// Reads an AnimationCurve subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_animation_curve_subnode(std::istream& is)
{
	int32 key_id, nb_keys = -1;
	AnimationCurve curve{};
	std::string subnode_key;
	char c;

	const auto on_size = [&](uint32 size){
		cgogn_assert(nb_keys == -1 || static_cast<uint32>(nb_keys) == size);
		nb_keys = size;
		curve.animation.reserve(nb_keys);
	};

	const auto get_keyframe = [&](const uint32& i) -> geometry::AnimationKeyframe<AnimTimeT, AnimScalar>& {
		if (curve.animation.size() <= i)
			curve.animation.resize(i + 1, {0.0, 0.0});
		return curve.animation[i];
	};

	read_object_attributes(is, curve.id);

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			animation_curves_.push_back(std::move(curve));
			return;
		case ':':
			key_id = 0;
			if (subnode_key == "KeyTime"s)
			{
				if (read_array(is, on_size,
				[&]{
					uint64 time;
					is >> time;
					get_keyframe(key_id++).time_ = time * ANIM_TIME_RATIO;
					return 1;
				}))
				{ cgogn_assert(key_id == nb_keys); }
				else
					std::cout << "Warning: invalid key times syntax" << std::endl;
			}
			else if (subnode_key == "KeyValueFloat"s)
			{
				if (read_array(is, on_size,
				[&]{
					FbxImporterBase::AnimScalar value;
					is >> value;
					get_keyframe(key_id++).value_ = value;
					return 1;
				}))
				{ cgogn_assert(key_id == nb_keys); }
				else
					std::cout << "Warning: invalid key values syntax" << std::endl;
			}
			else if (subnode_key == "Default"s)
				is >> curve.default_value;
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid animation curve syntax" << std::endl;
}

// Reads an AnimationCurveNode subnode (in an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_animation_curve_node_subnode(std::istream& is)
{
	AnimationCurveNode node{};
	std::string subnode_key;
	char c;

	read_object_attributes(is, node.id);

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			animation_curve_nodes_.push_back(std::move(node));
			return;
		case ':':
			if (subnode_key == "Properties70"s)
				read_properties_70_subnode(is, node.properties);
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid animation curve node syntax" << std::endl;
}

void FbxImporterBase::read_object_attributes(std::istream& is, ObjectId& id,
		std::string* name, std::string* type)
{
	enum Stage { START, NAME, MIDDLE, TYPE, END };
	int stage = START;
	char c;

	is >> id;

	while (stage < END && is.get(c))
	{
		switch (stage)
		{
		case START:
		case MIDDLE:
			if (c == '"')
				++stage;
			break;
		case NAME:
			if (c == '"') // TODO handle escape sequences
				++stage;
			else if (name)
				*name += c;
			break;
		case TYPE:
			if (c == '"') // TODO handle escape sequences
				++stage;
			else if (type)
				*type += c;
			break;
		}
	}
}

// Reads a Connections node from past the declaring colon through its closing brace
void FbxImporterBase::read_connections_node(std::istream& is)
{
	int commas = -1; // -1 is flag for not having read a colon
	ObjectId ids[2];
	std::string property_name;
	bool expect_property = false;
	bool in_quotes = false;
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0 && commas == -1); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0 && commas == -1);
			return;
		case '"':
			in_quotes = !in_quotes;
			if (!in_quotes)
			{
				if (!property_name.empty())
				{
					connections_op_.emplace_back(ids[0], ids[1], property_name);
					property_name.clear();
					expect_property = false;
					commas = -1;
				}
			}
			break;
		case ',':
			switch (commas)
			{
			case 0:
			case 1:
				is >> ids[commas];
				++commas;
				if (!expect_property && commas == 2)
				{
					connections_oo_.emplace_back(ids[0], ids[1]);
					commas = -1;
				}
				break;
			case 2:
				cgogn_assert(expect_property);
				++commas;
				break;
			default:
				cgogn_assert_not_reached("Previous commas not in [0, 2] like expected");
			}
			break;
		case ';':
			cgogn_assert(!in_quotes);
			skip_through_character(is, '\n');
			break;
		case 'P':
			if (commas == 0) // assume "OP"
			{
				cgogn_assert(in_quotes && c_ == 'O');
				expect_property = true;
				break;
			}
			// Intentional fallthrough
		default:
			if (in_quotes && expect_property)
			{
				cgogn_assert(commas == 3);
				property_name += c;
			}
			else if (!in_quotes && c == ':')
			{
				cgogn_assert(commas == -1 && c_ == 'C');
				commas = 0;
			}
#ifdef CGOGN_DEBUG
			else
				c_ = c;
#endif
		}
	}

	std::cout << "Warning: invalid syntax for Connections node" << std::endl;
}

// Reads a FBXHeaderExtension node from past the declaring colon through its closing brace
void FbxImporterBase::read_fbx_header_extension_node(std::istream& is)
{
	std::string subnode_key;
	char c;

	subnode_key.reserve(32);

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode without key
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return;
		case ':':
			if (subnode_key == "FBXVersion"s)
				read_integer_and_warn_if_not_expected(is, "FBX version", 7700);
			else
				skip_value(is);
			subnode_key.clear();
			break;
		case ';':
			if (subnode_key.empty())
				skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				subnode_key += c;
		}
	}

	std::cout << "Warning: invalid syntax for FBXHeaderExtension node" << std::endl;
}

// Reads a Definitions node from past the declaring colon through its closing brace
void FbxImporterBase::read_definitions_node(std::istream& is)
{
	skip_node(is); // TODO
}

// Reads a Properties70 subnode from past the declaring colon through its closing brace
void FbxImporterBase::read_properties_70_subnode(std::istream& is, Properties& p)
{
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return;
		case ':':
			cgogn_assert(c_ == 'P');
			read_property(is, p);
			c = '\0';
			break;
		case ';':
			cgogn_assert(c_ == '\0');
			skip_through_character(is, '\n');
			break;
		default:
			if (!std::isspace(c))
				c_ = c;
		}
	}

	std::cout << "Warning: invalid syntax for Properties70 subnode" << std::endl;
}

// Reads a property through the end of its line
void FbxImporterBase::read_property(std::istream& is, Properties& p)
{
	std::string name;
	char c;

	enum Stage { START, NAME, NAME_TO_TYPE, TYPE, TYPE_TO_TYPE_NAME, TYPE_NAME, TYPE_NAME_TO_LAST, LAST, END };
	int stage = START;

	while (stage < END && is.get(c))
	{
		switch (stage)
		{
		case START:
		case NAME_TO_TYPE:
		case TYPE_TO_TYPE_NAME:
		case TYPE_NAME_TO_LAST:
			if (c == '"')
				++stage;
			break;
		case NAME:
			if (c == '"') // TODO handle escape sequences
				++stage;
			else
				name += c;
			break;
		case TYPE:
		case TYPE_NAME:
		case LAST:
			if (c == '"') // TODO handle escape sequences
				++stage;
			break;
		}
	}

	auto pi = PROPERTY_INFO_.find(name);
	if (pi != PROPERTY_INFO_.cend())
	{
		const auto& [ptr, size] = pi->second(p);
		for (int i = 0; i < size && !is.fail() && skip_through_character(is, ','); ++i)
		{
			std::remove_reference_t<decltype(ptr[i].value())> value;
			is >> value;
			ptr[i].emplace(std::move(value));
		}
	}

	skip_through_character(is, '\n');
}

// Reads an array subnode from past the declaring colon through its closing braces
// `on_size` is provided with the number of scalars announced in the file
// `read_values` should return the amount of scalars read (normally 1)
// Returns whether the syntax was correct
bool FbxImporterBase::read_array(std::istream& is,
		std::function<void(uint32)> on_size, std::function<uint32()> read_values)
{
	bool in_array = false;
	int32 announced_size = -1, actual_size = 0;
	char c, c_ = '\0';

	for (int depth = 0; is.get(c);)
	{
		switch (c)
		{
		case '{':
			cgogn_assert(depth == 0); // no subnode
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			if (announced_size >= 0 && announced_size != actual_size)
				std::cout << "Warning: expected size " << announced_size
						<< " for array but it was actually of size " << actual_size << std::endl;
			return true;
		case ':':
			cgogn_assert(c_ == 'a' && depth == 1);
			in_array = true;
			c = '\0';
			// Intentional fallthrough
		case ',':
			cgogn_assert(in_array);
			actual_size += read_values();
			break;
		case ';':
			cgogn_assert(c_ == '\0');
			skip_through_character(is, '\n');
			break;
		case '\n':
			if (depth == 1)
				c = '\0'; // allow comment on new line inside braces
			break;
		default:
			if (depth == 0)
			{
				if (c == '*')
				{
					is >> announced_size;
					on_size(announced_size);
				}
			}
			else if (!in_array && !std::isspace(c))
				c_ = c;
		}
	}

	return false;
}

// Skips whitespace, reads an integer, and warns if it is not the expected value
void FbxImporterBase::read_integer_and_warn_if_not_expected(std::istream& is,
		const std::string& nature, int expected_value)
{
	int value;
	std::string value_str = "<ERROR>";

	if ((is >> value).fail())
		value = -expected_value;
	else
		value_str = std::to_string(value);

	if (value != expected_value)
		std::cout << "Warning: expected " << nature << " " << expected_value
				<< " but got " << value_str << std::endl;
}

// Ignores everything until after the next node's yet-unopened braces close
void FbxImporterBase::skip_node(std::istream& is)
{
	char c;
	for (int depth = 0; is.get(c);)
	{
		if (c == '{')
			++depth;
		else if (c == '}')
			--depth;
		else
			continue; // ignore any other character

		if (depth == 0) // back out
			return;
	}
}

// Ignores everything until after a line break or the end of the next node
void FbxImporterBase::skip_value(std::istream& is)
{
	char c;
	while (is.get(c))
	{
		if (c == '\n') // primitive
			return;

		if (c == '{') // node
		{
			is.unget();
			skip_node(is);
		}
	}
}

// Ignores everything through a given character
std::istream& FbxImporterBase::skip_through_character(std::istream& is, char c)
{
	return is.ignore(std::numeric_limits<std::streamsize>::max(), c);
}

FbxImporterBase::ObjectId FbxImporterBase::get_parent_id(const ObjectId& child_id) const
{
	const auto parent_connection_it =
		std::find_if(connections_oo_.cbegin(), connections_oo_.cend(),
				[=](const std::pair<ObjectId, ObjectId>& c) { return c.first == child_id; });

	return parent_connection_it != connections_oo_.cend() ? parent_connection_it->second : INVALID_INDEX;
}

geometry::Quaternion FbxImporterBase::from_euler(const std::array<std::optional<double>, 3>& xyz,
		const RotationOrder& rotation_order)
{
	static const std::array<Vec3, 3> axes = {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}};

	auto res = geometry::Quaternion::Identity();
	size_t ro = rotation_order;

	for (int i = 0; i < 3; ++i)
	{
		size_t axis = ro % RotationOrder::Base;
		ro /= RotationOrder::Base;

		if (xyz[axis])
			res = Eigen::AngleAxis<geometry::Quaternion::Scalar>(M_PI / 180.0 * xyz[axis].value(), axes[axis])
					* res;
	}

	return res;
}

void FbxImporterBase::LimbNodeModel::set_animation(const AnimationT* anim,
		const std::string& transform_property_name, const std::string& axis_property_name)
{
	size_t anim_id;

	// Get transformation type offset
	if (transform_property_name == "Lcl Translation"s)
		anim_id = 0;
	else if (transform_property_name == "Lcl Rotation"s)
		anim_id = 3;
	else
		return;

	// Get axis offset
	if (axis_property_name == "d|Z"s)
		anim_id += 2;
	else if (axis_property_name == "d|Y"s)
		++anim_id;
	else if (axis_property_name != "d|X"s)
		return; // not d|<axis>, ignore

	animation[anim_id] = anim;
}

bool FbxImporterBase::LimbNodeModel::has_component(const size_t& offset) const
{
	return animation[offset] || animation[offset + 1] || animation[offset + 2];
}

bool FbxImporterBase::LimbNodeModel::has_translation() const { return has_component(0); }
bool FbxImporterBase::LimbNodeModel::has_rotation() const { return has_component(3); }

std::optional<FbxImporterBase::AnimScalar> FbxImporterBase::LimbNodeModel::get_value(
		const size_t& index, const AnimTimeT& time) const
{
	return animation[index] ? animation[index]->get_value(time) : std::optional<AnimScalar>();
}

Vec3 FbxImporterBase::LimbNodeModel::get_translation_or(const AnimTimeT& time, const Vec3& default_value) const
{
	if (!has_translation())
		return default_value;

	return Vec3{get_value(0, time).value_or(0.0), get_value(1, time).value_or(0.0), get_value(2, time).value_or(0.0)};
}

geometry::Quaternion FbxImporterBase::LimbNodeModel::get_rotation_or(const AnimTimeT& time,
		const geometry::Quaternion& pre_rotation, const geometry::Quaternion& post_rotation,
		const geometry::Quaternion& default_value) const
{
	if (!has_rotation())
		return default_value;

	return pre_rotation
			* FbxImporterBase::from_euler(std::array<std::optional<double>, 3>
					{get_value(3, time), get_value(4, time), get_value(5, time)})
			* post_rotation;
}

} // namespace io

} // namespace cgogn
