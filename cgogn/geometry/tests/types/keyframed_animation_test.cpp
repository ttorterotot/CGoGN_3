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

#include <cgogn/geometry/types/animation/keyframed_animation.h>
#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/types/dual_quaternion.h>

namespace cgogn
{

class KeyframedAnimationTest : public ::testing::Test
{
public:
	template <typename T>
	static std::enable_if_t<std::is_floating_point_v<T>, bool>
	approx(const T& a, const T& b, const T& prec = Eigen::NumTraits<T>::dummy_precision())
	{
		const T& diff = b - a;
		return diff <= prec && -diff <= prec;
	}
};

// Test if keyframe extrapolation works as expected around a single key (constant value)

TEST_F(KeyframedAnimationTest, single_key_int)
{
	geometry::KeyframedAnimation<std::vector, double, int> anim;
	anim.emplace_back(-2.0, 3);
	EXPECT_TRUE(anim.get_value(-5.0) == 3);
	EXPECT_TRUE(anim.get_value(0.0) == 3);
	EXPECT_TRUE(anim.get_value(8.0) == 3);
}

TEST_F(KeyframedAnimationTest, single_key_float)
{
	geometry::KeyframedAnimation<std::vector, double, float> anim;
	anim.emplace_back(-2.0, 3.0f);
	EXPECT_TRUE(approx(anim.get_value(-5.0), 3.0f));
	EXPECT_TRUE(approx(anim.get_value(0.0), 3.0f));
	EXPECT_TRUE(approx(anim.get_value(8.0), 3.0f));
}

TEST_F(KeyframedAnimationTest, single_key_double)
{
	geometry::KeyframedAnimation<std::vector, double, double> anim;
	anim.emplace_back(-2.0, 3.0);
	EXPECT_TRUE(approx(anim.get_value(-5.0), 3.0));
	EXPECT_TRUE(approx(anim.get_value(0.0), 3.0));
	EXPECT_TRUE(approx(anim.get_value(8.0), 3.0));
}

// Test if keyframe extrapolation works as expected around three keys (constant value)
// and if interpolation does too in-between they keys (linear interpolation)

TEST_F(KeyframedAnimationTest, three_keys_int)
{
	geometry::KeyframedAnimation<std::vector, double, int> anim;
	anim.emplace_back(-2.0, 3);
	anim.emplace_back(0.0, -1);
	anim.emplace_back(4.0, 0);
	EXPECT_TRUE(anim.get_value(-5.0) == 3);
	// Interpolation in floating point space is required to avoid integer arithmetic
	EXPECT_TRUE(approx(anim.get_value<double>(-2.0), 3.0));
	EXPECT_TRUE(approx(anim.get_value<double>(-1.0), 1.0));
	EXPECT_TRUE(approx(anim.get_value<double>(0.0), -1.0));
	EXPECT_TRUE(approx(anim.get_value<double>(1.0), -0.75));
	EXPECT_TRUE(approx(anim.get_value<double>(4.0), 0.0));
	EXPECT_TRUE(approx(anim.get_value<double>(8.0), 0.0));
}

TEST_F(KeyframedAnimationTest, three_keys_float)
{
	geometry::KeyframedAnimation<std::vector, double, float> anim;
	anim.emplace_back(-2.0, 3.0f);
	anim.emplace_back(0.0, -1.0f);
	anim.emplace_back(4.0, 0.0f);
	EXPECT_TRUE(anim.get_value(-5.0) == 3);
	EXPECT_TRUE(approx(anim.get_value(-2.0), 3.0f));
	EXPECT_TRUE(approx(anim.get_value(-1.0), 1.0f));
	EXPECT_TRUE(approx(anim.get_value(0.0), -1.0f));
	EXPECT_TRUE(approx(anim.get_value(1.0), -0.75f));
	EXPECT_TRUE(approx(anim.get_value(4.0), 0.0f));
	EXPECT_TRUE(approx(anim.get_value(8.0), 0.0f));
}

TEST_F(KeyframedAnimationTest, three_keys_double)
{
	geometry::KeyframedAnimation<std::vector, double, double> anim;
	anim.emplace_back(-2.0, 3.0);
	anim.emplace_back(0.0, -1.0);
	anim.emplace_back(4.0, 0.0);
	EXPECT_TRUE(anim.get_value(-5.0) == 3);
	EXPECT_TRUE(approx(anim.get_value(-2.0), 3.0));
	EXPECT_TRUE(approx(anim.get_value(-1.0), 1.0));
	EXPECT_TRUE(approx(anim.get_value(0.0), -1.0));
	EXPECT_TRUE(approx(anim.get_value(1.0), -0.75));
	EXPECT_TRUE(approx(anim.get_value(4.0), 0.0));
	EXPECT_TRUE(approx(anim.get_value(8.0), 0.0));
}

TEST_F(KeyframedAnimationTest, three_keys_vector2f)
{
	using V = Eigen::Vector2f;
	geometry::KeyframedAnimation<std::vector, double, V> anim;
	anim.emplace_back(-2.0, V{3.0, -1.0});
	anim.emplace_back(0.0, V{-1.0, -0.5});
	anim.emplace_back(4.0, V{0.0, 3.5});
	EXPECT_TRUE(anim.get_value(-5.0).isApprox(V{3.0, -1.0}));
	EXPECT_TRUE(anim.get_value(-2.0).isApprox(V{3.0, -1.0}));
	EXPECT_TRUE(anim.get_value(-1.0).isApprox(V{1.0, -0.75}));
	EXPECT_TRUE(anim.get_value(0.0).isApprox(V{-1.0, -0.5}));
	EXPECT_TRUE(anim.get_value(1.0).isApprox(V{-0.75, 0.5}));
	EXPECT_TRUE(anim.get_value(4.0).isApprox(V{0.0, 3.5}));
	EXPECT_TRUE(anim.get_value(8.0).isApprox(V{0.0, 3.5}));
}

TEST_F(KeyframedAnimationTest, three_keys_vector3d)
{
	using V = Eigen::Vector3d;
	geometry::KeyframedAnimation<std::vector, double, V> anim;
	anim.emplace_back(-2.0, V{3.0, 0.0, -1.0});
	anim.emplace_back(0.0, V{-1.0, 0.0, -0.5});
	anim.emplace_back(4.0, V{0.0, 0.0, 3.5});
	EXPECT_TRUE(anim.get_value(-5.0).isApprox(V{3.0, 0.0, -1.0}));
	EXPECT_TRUE(anim.get_value(-2.0).isApprox(V{3.0, 0.0, -1.0}));
	EXPECT_TRUE(anim.get_value(-1.0).isApprox(V{1.0, 0.0, -0.75}));
	EXPECT_TRUE(anim.get_value(0.0).isApprox(V{-1.0, 0.0, -0.5}));
	EXPECT_TRUE(anim.get_value(1.0).isApprox(V{-0.75, 0.0, 0.5}));
	EXPECT_TRUE(anim.get_value(4.0).isApprox(V{0.0, 0.0, 3.5}));
	EXPECT_TRUE(anim.get_value(8.0).isApprox(V{0.0, 0.0, 3.5}));
}

TEST_F(KeyframedAnimationTest, two_keys_rigid_transformation_2D)
{
	using RT = geometry::RigidTransformation<Eigen::Rotation2Dd, Eigen::Vector2d>;
	geometry::KeyframedAnimation<std::vector, double, RT> anim;
	const auto& test_for = [&anim](double time, bool rot_aff_tra, Eigen::Isometry2d expected)
	{
		return anim.get_value(time, identity_c, RT::interpolate)
				.to_transform(rot_aff_tra).isApprox(expected);
	};
	Eigen::Rotation2Dd r0{-0.25}, r1{0.75}, rh{0.25};
	Eigen::Vector2d v0{3.0, -1.0}, v1{-1.0, -0.5};
	Eigen::Translation2d t0{v0}, t1{v1}, th{1.0, -0.75};
	anim.emplace_back(-2.0, RT{r0, v0});
	anim.emplace_back(0.0, RT{r1, v1});
	// Rotation shouldn't affect translation
	EXPECT_TRUE(test_for(-5.0, false, t0 * r0));
	EXPECT_TRUE(test_for(-2.0, false, t0 * r0));
	EXPECT_TRUE(test_for(-1.0, false, th * rh));
	EXPECT_TRUE(test_for(0.0, false, t1 * r1));
	EXPECT_TRUE(test_for(1.0, false, t1 * r1));
	// Rotation should affect translation
	EXPECT_TRUE(test_for(-5.0, true, r0 * t0));
	EXPECT_TRUE(test_for(-2.0, true, r0 * t0));
	EXPECT_TRUE(test_for(-1.0, true, rh * th));
	EXPECT_TRUE(test_for(0.0, true, r1 * t1));
	EXPECT_TRUE(test_for(1.0, true, r1 * t1));
	// Test out false positives (non-interpolated cases already covered in RigidTransformation tests)
	EXPECT_FALSE(test_for(-1.0, false, rh * th));
	EXPECT_FALSE(test_for(-1.0, true, th * rh));
}

TEST_F(KeyframedAnimationTest, two_keys_rigid_transformation_3D)
{
	using RT = geometry::RigidTransformation<Eigen::Quaterniond, Eigen::Vector3d>;
	geometry::KeyframedAnimation<std::vector, double, RT> anim;
	const auto& test_for = [&anim](double time, bool rot_aff_tra, Eigen::Isometry3d expected)
	{
		return anim.get_value(time, identity_c, RT::interpolate)
				.to_transform(rot_aff_tra).isApprox(expected);
	};
	auto r0 = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 1, 0});
	auto r1 = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d{0, 0, -1}, Eigen::Vector3d{-1, 0, 0});
	auto rh = r0.slerp(0.5, r1);
	Eigen::Vector3d v0{3.0, 0.0, -1.0}, v1{-1.0, 0.0, -0.5};
	Eigen::Translation3d t0{v0}, t1{v1}, th{1.0, 0.0, -0.75};
	anim.emplace_back(-2.0, RT{r0, v0});
	anim.emplace_back(0.0, RT{r1, v1});
	// Rotation shouldn't affect translation
	EXPECT_TRUE(test_for(-5.0, false, t0 * r0));
	EXPECT_TRUE(test_for(-2.0, false, t0 * r0));
	EXPECT_TRUE(test_for(-1.0, false, th * rh));
	EXPECT_TRUE(test_for(0.0, false, t1 * r1));
	EXPECT_TRUE(test_for(1.0, false, t1 * r1));
	// Rotation should affect translation
	EXPECT_TRUE(test_for(-5.0, true, r0 * t0));
	EXPECT_TRUE(test_for(-2.0, true, r0 * t0));
	EXPECT_TRUE(test_for(-1.0, true, rh * th));
	EXPECT_TRUE(test_for(0.0, true, r1 * t1));
	EXPECT_TRUE(test_for(1.0, true, r1 * t1));
	// Test out false positives (non-interpolated cases already covered in RigidTransformation tests)
	EXPECT_FALSE(test_for(-1.0, false, rh * th));
	EXPECT_FALSE(test_for(-1.0, true, th * rh));
}

TEST_F(KeyframedAnimationTest, two_keys_dq)
{
	using namespace geometry; // conciseness
	KeyframedAnimation<std::vector, double, DualQuaternion> anim;
	const auto& test_for = [&anim](double time, DualQuaternion expected)
	{
		return anim.get_value(time).isApprox(expected);
	};
	auto r0 = Quaternion::FromTwoVectors(Vec3{1, 0, 0}, Vec3{0, 1, 0});
	auto r1 = Quaternion::FromTwoVectors(Vec3{0, 0, -1}, Vec3{-1, 0, 0});
	auto rh = r0.slerp(0.5, r1);
	Vec3 v0{3.0, 0.0, -1.0}, v1{-1.0, 0.0, -0.5};
	// t * r
	auto dq0 = DualQuaternion::from_tr(v0, r0);
	auto dq1 = DualQuaternion::from_tr(v1, r1);
	anim.emplace_back(-2.0, dq0);
	anim.emplace_back(0.0, dq1);
	EXPECT_TRUE(test_for(-5.0, dq0));
	EXPECT_TRUE(test_for(-2.0, dq0));
	EXPECT_TRUE(anim.get_value(-1.0).rotation().normalized().isApprox(rh)); // (*)
	EXPECT_TRUE(test_for(0.0, dq1));
	EXPECT_TRUE(test_for(1.0, dq1));
	anim.clear();
	// r * t
	dq0 = DualQuaternion::from_tr(v0, r0);
	dq1 = DualQuaternion::from_tr(v1, r1);
	anim.emplace_back(-2.0, dq0);
	anim.emplace_back(0.0, dq1);
	EXPECT_TRUE(test_for(-5.0, dq0));
	EXPECT_TRUE(test_for(-2.0, dq0));
	EXPECT_TRUE(anim.get_value(-1.0).rotation().normalized().isApprox(rh)); // (*)
	EXPECT_TRUE(test_for(0.0, dq1));
	EXPECT_TRUE(test_for(1.0, dq1));
	// (*) by the nature of dual quaternions, the dual part is interpolated non-linearly,
	//     but the real part undergoes spherical linear interpolation
}

TEST_F(KeyframedAnimationTest, sort)
{
	geometry::KeyframedAnimation<std::vector, double, double> anim;
	anim.emplace_back(2.0, 1.0);
	anim.emplace_back(-4.0, 0.0);
	anim.sort(false);
	EXPECT_TRUE(approx(anim.get_value(-8.0), 0.0));
	EXPECT_TRUE(approx(anim.get_value(0.5), 0.75));
	EXPECT_TRUE(approx(anim.get_value(8.0), 1.0));
}

TEST_F(KeyframedAnimationTest, sort_stable)
{
	geometry::KeyframedAnimation<std::vector, double, double> anim;
	anim.emplace_back(2.0, 1.0); // expected to affect interpolation
	anim.emplace_back(2.0, -4.0); // expected to affect extrapolation
	anim.emplace_back(-4.0, 4.0); // expected to affect extrapolation
	anim.emplace_back(-4.0, 0.0); // expected to affect interpolation
	anim.sort(true);
	EXPECT_TRUE(approx(anim.get_value(-8.0), 4.0));
	EXPECT_TRUE(approx(anim.get_value(0.5), 0.75));
	EXPECT_TRUE(approx(anim.get_value(8.0), -4.0));
}

} // namespace cgogn
