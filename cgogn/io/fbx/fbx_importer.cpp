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
	skip_node(is); // TODO
}

// Reads a Connections node from past the declaring colon through its closing brace
void FbxImporterBase::read_connections_node(std::istream& is)
{
	skip_node(is); // TODO
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
