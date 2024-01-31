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
using Vec3 = geometry::Vec3;
using Quaternion = geometry::Quaternion;
using DualQuaternion = geometry::DualQuaternion;

class DualQuaternionTest : public ::testing::Test
{
public:
	constexpr static geometry::Scalar prec = Eigen::NumTraits<geometry::Scalar>::dummy_precision();
};

TEST_F(DualQuaternionTest, identity)
{
	EXPECT_TRUE((DualQuaternion::identity().rotation().isApprox(Quaternion{1, 0, 0, 0})));
	EXPECT_TRUE((DualQuaternion::identity().translation().squaredNorm() <= prec * prec));
}

TEST_F(DualQuaternionTest, isApprox)
{
	DualQuaternion a = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0}, {256.0, 0.0, -256.0});
	DualQuaternion b = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0}, {256.0, 0.0, -256.0});
	DualQuaternion c = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0},
			{256.0, 0.0, -256.0 + 64.0 * prec}); // precision is relative, this is a quarter of the threshold
	DualQuaternion d = DualQuaternion::from_rt({0.0625, 1.0, 16.0, 1024.0},
			{256.0, 0.0, -256.0 + 1024.0 * prec}); // precision is relative, this is four times the threshold
	DualQuaternion y = DualQuaternion::from_translation({4.0 * prec, 0.0, 0.0});
	DualQuaternion z = DualQuaternion::from_translation({0.25 * prec, 0.0, 0.0});

	EXPECT_FALSE(a.isApprox(DualQuaternion::identity()));
	EXPECT_TRUE(a.isApprox(b));
	EXPECT_TRUE(a.isApprox(c, prec));
	EXPECT_FALSE(a.isApprox(d, prec));
	EXPECT_FALSE(y.isApprox(DualQuaternion::identity(), prec));
	EXPECT_TRUE(z.isApprox(DualQuaternion::identity(), prec));
}

TEST_F(DualQuaternionTest, from_rotation)
{
	Quaternion r{1.0, 2.0, 4.0, 8.0};
	DualQuaternion q = DualQuaternion::from_rotation(r);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized()));
	EXPECT_TRUE(q.translation().squaredNorm() <= prec * prec);
}

TEST_F(DualQuaternionTest, from_translation)
{
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_translation(t);
	EXPECT_TRUE(q.rotation().isApprox(Quaternion{1, 0, 0, 0}));
	EXPECT_TRUE(q.translation().isApprox(t));
}

TEST_F(DualQuaternionTest, from_rt)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	Vec3 t_ = r.normalized()._transformVector(t);
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized()));
	EXPECT_TRUE(q.translation().isApprox(t_));
}

TEST_F(DualQuaternionTest, from_tr)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_tr(t, r);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized()));
	EXPECT_TRUE(q.translation().isApprox(t));
}

TEST_F(DualQuaternionTest, identity_rt)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE(q.isApprox(q * DualQuaternion::identity()));
	EXPECT_TRUE(q.isApprox(DualQuaternion::identity() * q));
}

TEST_F(DualQuaternionTest, identity_tr)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_tr(t, r);
	EXPECT_TRUE(q.isApprox(q * DualQuaternion::identity()));
	EXPECT_TRUE(q.isApprox(DualQuaternion::identity() * q));
}

TEST_F(DualQuaternionTest, normalized)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t) * 2.0;
	EXPECT_FALSE(almost_equal_relative(q.magnitude(), geometry::Scalar(1)));
	EXPECT_TRUE(almost_equal_relative(q.normalized().magnitude(), geometry::Scalar(1)));;
}

TEST_F(DualQuaternionTest, normalize_normalized)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	DualQuaternion q_ = q;
	q_.normalize();
	EXPECT_TRUE(q.normalized().isApprox(q_));
}

TEST_F(DualQuaternionTest, conjugated)
{
	Quaternion r{1.0, 2.0, 4.0, 8.0};
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE((q * q.conjugated()).isApprox(q.conjugated() * q));
	EXPECT_FALSE(DualQuaternion::identity().isApprox(q * q.conjugated()));
	EXPECT_FALSE(DualQuaternion::identity().isApprox(q.conjugated() * q));
}

TEST_F(DualQuaternionTest, conjugate_conjugated)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	DualQuaternion q_ = q;
	q_.conjugate();
	EXPECT_TRUE(q.conjugated().isApprox(q_));
}

} // namespace cgogn