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

// Test if DualQuaternion::identity is indeed the identity DQ
TEST_F(DualQuaternionTest, identity)
{
	EXPECT_TRUE((DualQuaternion::identity().rotation().isApprox(Quaternion{1, 0, 0, 0})));
	EXPECT_TRUE((DualQuaternion::identity().translation().squaredNorm() <= prec * prec));
}

// Test if DualQuaternion::isApprox is complete and correct, especially in regards to precision
// Attention is brought to false positives because we use this method to check results in later tests
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

	EXPECT_FALSE(a.isApprox(DualQuaternion::identity())); // clearly different values
	EXPECT_TRUE(a.isApprox(b)); // same value
	EXPECT_TRUE(a.isApprox(c, prec)); // close enough value
	EXPECT_FALSE(a.isApprox(d, prec)); // too far apart
	EXPECT_FALSE(y.isApprox(DualQuaternion::identity(), prec)); // too far apart
	EXPECT_TRUE(z.isApprox(DualQuaternion::identity(), prec)); // close enough value
}

// Test if DualQuaternion::from_rotation initializes the DQ properly
TEST_F(DualQuaternionTest, from_rotation)
{
	Quaternion r{1.0, 2.0, 4.0, 8.0};
	DualQuaternion q = DualQuaternion::from_rotation(r);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized())); // (normalized) rotation preserved
	EXPECT_TRUE(q.translation().squaredNorm() <= prec * prec); // no translation
}

// Test if DualQuaternion::from_translation initializes the DQ properly
TEST_F(DualQuaternionTest, from_translation)
{
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_translation(t);
	EXPECT_TRUE(q.rotation().isApprox(Quaternion{1, 0, 0, 0})); // no rotation
	EXPECT_TRUE(q.translation().isApprox(t)); // translation preserved
}

// Test if DualQuaternion::from_point initializes the DQ properly
TEST_F(DualQuaternionTest, from_point)
{
	Vec3 p{1.0, 2.0, 4.0};
	EXPECT_TRUE(DualQuaternion::from_point(p).point().isApprox(p)); // point preserved
}

// Test if DualQuaternion::from_rt initializes the DQ properly
TEST_F(DualQuaternionTest, from_rt)
{
	// r is already normalized but we pretend it isn't and still normalize it
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized())); // (normalized) rotation preserved
	EXPECT_TRUE(q.translation().isApprox(r.normalized()._transformVector(t))); // translation rotated
}

// Test if DualQuaternion::from_tr initializes the DQ properly
TEST_F(DualQuaternionTest, from_tr)
{
	// r is already normalized but we pretend it isn't and still normalize it
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_tr(t, r);
	EXPECT_TRUE(q.rotation().isApprox(r.normalized())); // (normalized) rotation preserved
	EXPECT_TRUE(q.translation().isApprox(t)); // translation preserved
}

// Test if DualQuaternion::identity is indeed identity (with DQ created with from_rt)
TEST_F(DualQuaternionTest, identity_rt)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE(q.isApprox(q * DualQuaternion::identity())); // as right operand
	EXPECT_TRUE(q.isApprox(DualQuaternion::identity() * q)); // as left operand
}

// Test if DualQuaternion::identity is indeed identity (with DQ created with from_tr)
TEST_F(DualQuaternionTest, identity_tr)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_tr(t, r);
	EXPECT_TRUE(q.isApprox(q * DualQuaternion::identity())); // as right operand
	EXPECT_TRUE(q.isApprox(DualQuaternion::identity() * q)); // as left operand
}

// Test if DualQuaternion::normalized can normalize a non-normalized DQ
TEST_F(DualQuaternionTest, normalized)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t) * 2.0;
	EXPECT_FALSE(almost_equal_relative(q.magnitude(), geometry::Scalar(1))); // scaled not normalized
	EXPECT_TRUE(almost_equal_relative(q.normalized().magnitude(), geometry::Scalar(1)));
}

// Test if DualQuaternion::normalize is indeed the in-place equivalent of DualQuaternion::normalized
TEST_F(DualQuaternionTest, normalize_normalized)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	DualQuaternion q_ = q;
	q_.normalize();
	EXPECT_TRUE(q.normalized().isApprox(q_));
}

// Test if DualQuaternion::isNormalized is correct
TEST_F(DualQuaternionTest, isNormalized)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t) * 2.0;
	EXPECT_FALSE(q.isNormalized()); // scaled not normalized
	EXPECT_TRUE(q.normalized().isNormalized());
}

// Test if DualQuaternion::conjugated is indeed the conjugate: q q* == q* q != identity
// We check it's not the identity because we don't expect it to be the case for this value of q
// and it would imply q q* == q* q by itself (informally, if q q* != q* q == identity,
// what would even be the value of q q* given it intuitively remains a sort of "mirror" of q* q?)
TEST_F(DualQuaternionTest, conjugated)
{
	Quaternion r{1.0, 2.0, 4.0, 8.0};
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	EXPECT_TRUE((q * q.conjugated()).isApprox(q.conjugated() * q)); // q q* == q* q
	EXPECT_FALSE(DualQuaternion::identity().isApprox(q * q.conjugated())); // q q* != identity
	EXPECT_FALSE(DualQuaternion::identity().isApprox(q.conjugated() * q)); // q* q != identity
}

// Test if DualQuaternion::conjugate is indeed the in-place equivalent of DualQuaternion::conjugated
TEST_F(DualQuaternionTest, conjugate_conjugated)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 2.0, 4.0}, Vec3{-1.0, -0.5, -0.25});
	Vec3 t{1.0, 2.0, 4.0};
	DualQuaternion q = DualQuaternion::from_rt(r, t);
	DualQuaternion q_ = q;
	q_.conjugate();
	EXPECT_TRUE(q.conjugated().isApprox(q_));
}

// Test if DualQuaternion::identity's transform is indeed a no-op
TEST_F(DualQuaternionTest, transform_identity)
{
	Vec3 p{1.0, 2.0, 4.0};
	EXPECT_TRUE(DualQuaternion::identity().transform(p).isApprox(p));
}

// Test if DualQuaternion::transform produces the expected results
TEST_F(DualQuaternionTest, transform_rt_tr)
{
	Quaternion r = Quaternion::FromTwoVectors(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}); // pi/2 around Z
	Vec3 t{0.0, 1.0, 1.0}; // translation
	Vec3 p{-1.0, -1.0, 0.0}; // initial point
	// (-1, -1, 0) --t-> (-1, 0, 1) --r-> (0, -1, 1)
	EXPECT_TRUE(DualQuaternion::from_rt(r, t).transform(p).isApprox(Vec3{0.0, -1.0, 1.0}));
	// (-1, -1, 0) --r-> (1, -1, 0) --t-> (1, 0, 1)
	EXPECT_TRUE(DualQuaternion::from_tr(t, r).transform(p).isApprox(Vec3{1.0, 0.0, 1.0}));
}

} // namespace cgogn
