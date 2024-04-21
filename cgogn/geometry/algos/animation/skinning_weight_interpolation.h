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

#ifndef CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_
#define CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_

#include <optional>
#include <vector>

#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/core/utils/assert.h>

namespace cgogn
{

namespace geometry
{

class SkinningWeightInterpolation
{
public:
	// None of these both preserve partial weights and compensate for weight lost due to the limit of four
	// Attempting to do both while supporting negative weights would require handling a number of edge cases
	enum class NormalizationType
	{
		None, // don't normalize, summed weights might exceed 1
		Sum, // divide by weight sum (result sum: 1)
		AbsSum, // divide by weight absolute sum (result sum: 1 or -1)
		CellCount, // divide by amount of contributing cells
	};

public:

	SkinningWeightInterpolation() = delete;

	template <typename MESH, typename CONT_C, typename CONT_W = std::array<Vec4::Scalar, 0>>
	static std::pair<Vec4i, Vec4> compute_weights(const MESH& m,
			const typename mesh_traits<MESH>::template Attribute<Vec4i>& weight_index,
			const typename mesh_traits<MESH>::template Attribute<Vec4>& weight_value,
			const CONT_C& source_cells,
			const std::optional<CONT_W> source_cell_weights = {},
			std::vector<Vec4::Scalar>& weight_value_buffer = {},
			const NormalizationType& normalization = NormalizationType::AbsSum)
	{
		Vec4i indices{ -1, -1, -1, -1 };
		Vec4 values = Vec4::Zero();
		weight_value_buffer.resize(0);

		if (source_cells.empty())
			return std::make_pair(indices, values);

		// Sum up all weight contributions per-bone into the buffer
		for (size_t i = 0; i < source_cells.size(); ++i)
		{
			const auto& vi = index_of(m, source_cells[i]);
			const auto vw = source_cell_weights ? (*source_cell_weights)[i] : 1.0;
			for (size_t i = 0; i < 4; ++i)
			{
				const int wi = weight_index[vi][i];
				const double wv = vw * weight_value[vi][i];
				if (wi < 0 || wv == 0.0)
					continue;
				if (weight_value_buffer.size() <= wi)
					weight_value_buffer.resize(wi + 1, 0.0);
				weight_value_buffer[wi] += wv;
			}
		}

		// Find and set four highest weight values, and set corresponding indices
		for (size_t wi = 0; wi < weight_value_buffer.size(); ++wi)
		{
			const auto& wv = weight_value_buffer[wi];
			if (wv == 0.0) // zero weight, not worth adding
				continue;
			int target_coef = -1; // vector index to write the new weight to
			for (size_t i = 0; i < 4; ++i)
			{
				if (indices[i] == -1) // free space, put here
				{
					target_coef = i;
					break;
				}
				if (wv < values[i]) // higher value, don't put here
					continue;
				if (target_coef == -1 // no target yet, so this is the best fit
						|| values[i] < values[target_coef]) // better fit than previous target
					target_coef = i;
			}
			if (target_coef == -1) // all weights already higher than wv
				continue;
			indices[target_coef] = wi;
			values[target_coef] = wv;
		}

		switch (normalization)
		{
		case NormalizationType::None:
			break;
		case NormalizationType::Sum:
			if (!values.isZero())
				values /= values.sum();
			break;
		case NormalizationType::AbsSum:
			if (!values.isZero())
				values /= std::abs(values.sum());
			break;
		case NormalizationType::CellCount:
			// source_cells.size() > 0 due to early return above
			values /= source_cells.size();
			break;
		default:
			cgogn_assert_not_reached("Missing normalization type case");
		}

		return std::make_pair(indices, values);
	}
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_SKINNING_WEIGHT_INTERPOLATION_H_
