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

#include <cgogn/core/types/animation/animation_skeleton.h>

#define STRONG_CHECKS true

namespace cgogn
{
class AnimationSkeletonTest : public ::testing::Test
{
protected:
	AnimationSkeleton as;

	uint32 index_of(const AnimationSkeleton::Bone& bone)
	{
		return std::find(as.bone_traverser_.cbegin(), as.bone_traverser_.cend(), bone) - as.bone_traverser_.cbegin();
	}
};

// Test if attach_bone preserves bone traverser order conditions
// parent <- child    ->    parent <- child     ->    parent <- insert <- child
// insert                          ^- insert
TEST_F(AnimationSkeletonTest, attach_bone_line)
{
	auto parent = add_root(as);
	auto child = add_bone(as, parent);
	auto insert = add_root(as);

	EXPECT_TRUE(index_of(parent) < index_of(child));

#if STRONG_CHECKS
	EXPECT_TRUE(as.bone_traverser_[0] == parent);
	EXPECT_TRUE(as.bone_traverser_[1] == child);
	EXPECT_TRUE(as.bone_traverser_[2] == insert);
#endif

	attach_bone(as, insert, parent);
	attach_bone(as, child, insert);

	EXPECT_TRUE(index_of(parent) < index_of(insert));
	EXPECT_TRUE(index_of(insert) < index_of(child));

#if STRONG_CHECKS
	EXPECT_TRUE(as.bone_traverser_[0] == parent);
	EXPECT_TRUE(as.bone_traverser_[1] == insert);
	EXPECT_TRUE(as.bone_traverser_[2] == child);
#endif
}

// Test if attach_bone preserves bone traverser order conditions
// parent <- child_l <- grandchild_l          new_root <- parent <- child_l <- grandchild_l
//        ^- child_r <- grandchild_l    ->                       ^- child_r <- grandchild_r
// new_root
TEST_F(AnimationSkeletonTest, attach_bone_triangle)
{
	auto parent = add_root(as);
	auto child_l = add_bone(as, parent);
	auto child_r = add_bone(as, parent);
	auto grandchild_l = add_bone(as, child_l);
	auto grandchild_r = add_bone(as, child_r);
	auto new_root = add_root(as);

	EXPECT_TRUE(index_of(parent) < index_of(child_l));
	EXPECT_TRUE(index_of(parent) < index_of(child_r));
	EXPECT_TRUE(index_of(child_l) < index_of(grandchild_l));
	EXPECT_TRUE(index_of(child_r) < index_of(grandchild_r));

#if STRONG_CHECKS
	EXPECT_TRUE(as.bone_traverser_[0] == parent);
	EXPECT_TRUE(as.bone_traverser_[1] == child_l);
	EXPECT_TRUE(as.bone_traverser_[2] == child_r);
	EXPECT_TRUE(as.bone_traverser_[3] == grandchild_l);
	EXPECT_TRUE(as.bone_traverser_[4] == grandchild_r);
	EXPECT_TRUE(as.bone_traverser_[5] == new_root);
#endif

	attach_bone(as, parent, new_root);

	EXPECT_TRUE(index_of(new_root) < index_of(parent));
	EXPECT_TRUE(index_of(parent) < index_of(child_l));
	EXPECT_TRUE(index_of(parent) < index_of(child_r));
	EXPECT_TRUE(index_of(child_l) < index_of(grandchild_l));
	EXPECT_TRUE(index_of(child_r) < index_of(grandchild_r));

#if STRONG_CHECKS
	EXPECT_TRUE(as.bone_traverser_[0] == new_root);
	EXPECT_TRUE(as.bone_traverser_[1] == parent);
	EXPECT_TRUE(as.bone_traverser_[2] == child_l);
	EXPECT_TRUE(as.bone_traverser_[3] == child_r);
	EXPECT_TRUE(as.bone_traverser_[4] == grandchild_l);
	EXPECT_TRUE(as.bone_traverser_[5] == grandchild_r);
#endif
}

// Test that attach_bone doesn't affect bone traverser order if unneeded
//    parent <- child <- grandchild | insert
// -> parent <- child <- (grandchild | insert)
// -> parent <- child <- grandchild <- insert
// -> parent <- (child <- grandchild | insert)
TEST_F(AnimationSkeletonTest, attach_bone_no_reorder)
{
	auto parent = add_root(as);
	auto child = add_bone(as, parent);
	auto grandchild = add_bone(as, parent);
	auto insert = add_root(as);

	const auto check = [&] {

		EXPECT_TRUE(index_of(parent) < index_of(child));
		EXPECT_TRUE(index_of(child) < index_of(grandchild));

#if STRONG_CHECKS
		EXPECT_TRUE(as.bone_traverser_[0] == parent);
		EXPECT_TRUE(as.bone_traverser_[1] == child);
		EXPECT_TRUE(as.bone_traverser_[2] == grandchild);
		EXPECT_TRUE(as.bone_traverser_[3] == insert);
#endif
	};

	check();

	attach_bone(as, insert, child);
	EXPECT_TRUE(index_of(child) < index_of(insert));
	check();

	attach_bone(as, insert, grandchild);
	EXPECT_TRUE(index_of(grandchild) < index_of(insert));
	check();

	attach_bone(as, insert, parent);
	EXPECT_TRUE(index_of(parent) < index_of(insert));
	check();
}

} // namespace cgogn
