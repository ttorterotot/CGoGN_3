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

#include <gtest/gtest.h>

#include <cgogn/geometry/types/dual_quaternion.h>

namespace cgogn
{
using DualQuaternion = geometry::DualQuaternion;

class DualQuaternionTest : public ::testing::Test
{
};

TEST_F(DualQuaternionTest, isApprox)
{
	DualQuaternion a = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0}, {256.0, 0.0, -256.0});
	DualQuaternion b = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0}, {256.0, 0.0, -256.0});

	EXPECT_FALSE(a.isApprox(DualQuaternion::identity()));
	EXPECT_TRUE(a.isApprox(b));
}

} // namespace cgogn