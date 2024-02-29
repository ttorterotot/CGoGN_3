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

	node_key.reserve(64);

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
			else
				skip_value(is); // TODO
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
}

// Reads a Model subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_model_subnode(std::istream& is)
{
	Model model{};
	std::string type;
	std::string subnode_key;
	char c;

	subnode_key.reserve(32);

	enum Stage { START, NAME, NAME_TO_TYPE, TYPE, END };
	int stage = START;

	is >> model.id;

	while (stage < END && is.get(c))
	{
		switch (stage)
		{
		case START:
		case NAME_TO_TYPE:
			if (c == '"')
				++stage;
			break;
		case NAME:
			if (c == '"') // TODO handle escape sequences
				++stage;
			else
				model.name += c;
			break;
		case TYPE:
			if (c == '"') // TODO handle escape sequences
				++stage;
			else
				type += c;
			break;
		}
	}

	if (type == "Mesh"s)
	{
		if (!load_surfaces_)
		{
			skip_node(is);
			return;
		}
		model.type = ModelType::Mesh;
	}
	else if (type == "LimbNode"s)
	{
		if (!load_skeletons_)
		{
			skip_node(is);
			return;
		}
		model.type = ModelType::LimbNode;
	}

	for (int depth = 0; stage == END && is.get(c);)
	{
		switch (c)
		{
		case '{':
			++depth;
			break;
		case '}':
			--depth;
			cgogn_assert(depth == 0);
			return;
		case ':':
			if (subnode_key == "Version"s)
				read_integer_and_warn_if_not_expected(is, "model version", 232);
			else if (subnode_key == "Properties70"s)
				read_properties_70_subnode(is, model.properties);
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

	models_.push_back(std::move(model));
}

// Reads a Geometry subnode (inside an Objects node) from past the declaring colon through its closing brace
void FbxImporterBase::read_objects_geometry_subnode(std::istream& is)
{
	skip_node(is); // TODO
}

// Reads a Connections node from past the declaring colon through its closing brace
void FbxImporterBase::read_connections_node(std::istream& is)
{
	skip_node(is); // TODO
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
			is >> ptr[i];
	}

	skip_through_character(is, '\n');
}

// Skips whitespace, reads an integer, and warns if it is not the expected value
void FbxImporterBase::read_integer_and_warn_if_not_expected(std::istream& is,
		const std::string& nature, int expected_value)
{
	int value;
	std::string value_str = "<ERROR>";

	if ((is >> value).fail())
		value == -expected_value;
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

} // namespace io

} // namespace cgogn
