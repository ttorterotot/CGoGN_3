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

#include <cgogn/geometry/types/rigid_transformation.h>
#include <cgogn/geometry/functions/quaternion_operations.h>

namespace cgogn
{

class RigidTransformationTest : public ::testing::Test
{
};

// Test if dimension is correctly inferred from template resolution
TEST_F(RigidTransformationTest, template_dim)
{
	// Implying namespaces to be concise
	using namespace Eigen;
	using geometry::RigidTransformation;
	// 2D vectors
	EXPECT_TRUE(RigidTransformation{Vector2f{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Vector2d{}}.Dim == 2);
	// 2D transformations
	EXPECT_TRUE(RigidTransformation{Rotation2Df{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Rotation2Dd{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Translation2f{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Translation2d{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Rotation2Df{} * Translation2f{}}.Dim == 2);
	EXPECT_TRUE(RigidTransformation{Rotation2Dd{} * Translation2d{}}.Dim == 2);
	// 3D/4D vectors and quaternions
	EXPECT_TRUE(RigidTransformation{Vector3f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Vector3d{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Vector4f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Vector4d{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Quaternionf{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Quaterniond{}}.Dim == 3);
	// 3D transformations
	EXPECT_TRUE(RigidTransformation{AngleAxisf{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{AngleAxisd{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Translation3f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Translation3d{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Quaternionf{} * Translation3f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Quaterniond{} * Translation3d{}}.Dim == 3);
	// Matrices (3D, regular or homogeneous)
	EXPECT_TRUE(RigidTransformation{Matrix3f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Matrix3d{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Matrix4f{}}.Dim == 3);
	EXPECT_TRUE(RigidTransformation{Matrix4d{}}.Dim == 3);
}

// Test if scalar is correctly inferred from template resolution
TEST_F(RigidTransformationTest, template_scalar)
{
	// Implying namespaces to be concise
	using namespace Eigen;
	using geometry::RigidTransformation;
	// 2D vectors
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector2d{}})::Scalar, double>));
	// 2D transformations
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Rotation2Df{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Rotation2Dd{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Translation2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Translation2d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Rotation2Df{} * Translation2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Rotation2Dd{} * Translation2d{}})::Scalar, double>));
	// 3D/4D vectors and quaternions
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector4f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Vector4d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Quaternionf{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Quaterniond{}})::Scalar, double>));
	// 3D transformations
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{AngleAxisf{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{AngleAxisd{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Translation3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Translation3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Quaternionf{} * Translation3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Quaterniond{} * Translation3d{}})::Scalar, double>));
	// Matrices (3D, regular or homogeneous)
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Matrix3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Matrix3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Matrix4f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(RigidTransformation{Matrix4d{}})::Scalar, double>));
}

// Test if RigidTransformation(Transform(Rotation2D)) initializes the RT properly
TEST_F(RigidTransformationTest, from_transform_rotation2)
{
	Eigen::Rotation2Dd rotation{0.25};
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Isometry2d{rotation}}.rotation().isApprox(rotation));
}

// Test if RigidTransformation(Transform(Quaternion)) initializes the RT properly
TEST_F(RigidTransformationTest, from_transform_rotation3)
{
	using namespace geometry; // required for operator*(Quatd, double), otherwise convenience
	Eigen::Quaterniond rotation{Eigen::AngleAxisd{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()}};
	// A quaternion and the same with opposite coefficients represent the same rotation
	// (the only difference is whether or not interpolation with another quaternion would take the shortest path)
	const bool& equal_direct = RigidTransformation{Eigen::Isometry3d{rotation}}.rotation().isApprox(rotation);
	const bool& equal_opposite = RigidTransformation{Eigen::Isometry3d{rotation}}.rotation().isApprox(rotation * -1.0);
	EXPECT_TRUE(equal_direct || equal_opposite);
}

// Test if RigidTransformation(Transform(Translation2)) initializes the RT properly
TEST_F(RigidTransformationTest, from_transform_translation2)
{
	Eigen::Vector2d translation{0.5, -1.0};
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Isometry2d{Eigen::Translation2d{translation}}}
			.translation().isApprox(translation));
}

// Test if RigidTransformation(Transform(Translation3)) initializes the RT properly
TEST_F(RigidTransformationTest, from_transform_translation3)
{
	Eigen::Vector3d translation{0.5, -1.0, 2.0};
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Isometry3d{Eigen::Translation3d{translation}}}
			.translation().isApprox(translation));
}

// Test if RigidTransformation(Rotation2D) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_rotation2D)
{
	Eigen::Rotation2Dd rotation{0.25};
	EXPECT_TRUE(geometry::RigidTransformation{rotation}.to_transform_matrix()
			.isApprox(Eigen::Isometry2d{rotation}.matrix()));
}

// Test if RigidTransformation(AngleAxis) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_angle_axis)
{
	Eigen::AngleAxisd rotation{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()};
	EXPECT_TRUE(geometry::RigidTransformation{rotation}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{rotation}.matrix()));
}

// Test if RigidTransformation(Quaternion) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_quaternion)
{
	Eigen::Quaterniond rotation{Eigen::AngleAxisd{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()}};
	EXPECT_TRUE(geometry::RigidTransformation{rotation}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{rotation}.matrix()));
}

// Test if RigidTransformation(Vector2) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_vector2)
{
	Eigen::Vector2d translation{0.5, -1.0};
	EXPECT_TRUE(geometry::RigidTransformation{translation}.to_transform_matrix()
			.isApprox(Eigen::Isometry2d{Eigen::Translation2d{translation}}.matrix()));
}

// Test if RigidTransformation(Vector3) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_vector3)
{
	Eigen::Vector3d translation{0.5, -1.0, 2.0};
	EXPECT_TRUE(geometry::RigidTransformation{translation}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{Eigen::Translation3d{translation}}.matrix()));
}

// Test if RigidTransformation(Vector4) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_vector4)
{
	Eigen::Vector4d translation_h{0.5, -1.0, 2.0, 4.0};
	Eigen::Vector3d translation = translation_h.head<3>() / translation_h.w();
	// Expecting homogeneous vector to be converted to its 3D form properly
	EXPECT_TRUE(geometry::RigidTransformation{translation_h}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{Eigen::Translation3d{translation}}.matrix()));
	// Expecting improperly-converted vector to produce different results
	EXPECT_FALSE(geometry::RigidTransformation{translation_h}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{Eigen::Translation3d{translation_h.head<3>()}}.matrix()));
}

// Test if RigidTransformation(Translation2) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_translation2)
{
	Eigen::Translation2d translation{0.5, -1.0};
	EXPECT_TRUE(geometry::RigidTransformation{translation}.to_transform_matrix()
			.isApprox(Eigen::Isometry2d{translation}.matrix()));
}

// Test if RigidTransformation(Translation3) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_translation3)
{
	Eigen::Translation3d translation{0.5, -1.0, 2.0};
	EXPECT_TRUE(geometry::RigidTransformation{translation}.to_transform_matrix()
			.isApprox(Eigen::Isometry3d{translation}.matrix()));
}

// Test if RigidTransformation(Rotation2D, Vector2) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_r2v2)
{
	using T = Eigen::Isometry2d; // conciseness
	Eigen::Rotation2Dd rotation{0.25};
	Eigen::Vector2d v{0.5, -1.0};
	Eigen::Translation2d translation{v};
	// Rotation shouldn't affect translation
	EXPECT_TRUE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(false)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_FALSE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(false)
			.isApprox((T{rotation} * T{translation}).matrix())));
	// Rotation should affect translation
	EXPECT_FALSE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(true)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_TRUE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(true)
			.isApprox((T{rotation} * T{translation}).matrix())));
}

// Test if RigidTransformation(Rotation2D, Translation2) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_r2t2)
{
	using T = Eigen::Isometry2d; // conciseness
	Eigen::Rotation2Dd rotation{0.25};
	Eigen::Translation2d translation{0.5, -1.0};
	// Rotation shouldn't affect translation
	EXPECT_TRUE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(false)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_FALSE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(false)
			.isApprox((T{rotation} * T{translation}).matrix())));
	// Rotation should affect translation
	EXPECT_FALSE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(true)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_TRUE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(true)
			.isApprox((T{rotation} * T{translation}).matrix())));
}

// Test if RigidTransformation(AngleAxis, Translation3) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_at3)
{
	using T = Eigen::Isometry3d; // conciseness
	Eigen::AngleAxisd rotation{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()};
	Eigen::Translation3d translation{0.5, -1.0, 2.0};
	// Rotation shouldn't affect translation
	EXPECT_TRUE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(false)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_FALSE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(false)
			.isApprox((T{rotation} * T{translation}).matrix())));
	// Rotation should affect translation
	EXPECT_FALSE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(true)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_TRUE((geometry::RigidTransformation{rotation, translation}.to_transform_matrix(true)
			.isApprox((T{rotation} * T{translation}).matrix())));
}

// Test if RigidTransformation(Quaternion, Vector3) outputs the correct transformation matrix
TEST_F(RigidTransformationTest, matrix_from_qv3)
{
	using T = Eigen::Isometry3d; // conciseness
	Eigen::Quaterniond rotation{Eigen::AngleAxisd{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()}};
	Eigen::Vector3d v{0.5, -1.0, 2.0};
	Eigen::Translation3d translation{v};
	// Rotation shouldn't affect translation
	EXPECT_TRUE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(false)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_FALSE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(false)
			.isApprox((T{rotation} * T{translation}).matrix())));
	// Rotation should affect translation
	EXPECT_FALSE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(true)
			.isApprox((T{translation} * T{rotation}).matrix())));
	EXPECT_TRUE((geometry::RigidTransformation{rotation, v}.to_transform_matrix(true)
			.isApprox((T{rotation} * T{translation}).matrix())));
}

// Test if RigidTransformation(Rotation2D, Vector2)::inverse is indeed the inverse: t t^-1 == t^-1 t == identity
TEST_F(RigidTransformationTest, inverse2)
{
	Eigen::Rotation2Dd rotation{0.25};
	Eigen::Vector2d translation{0.5, -1.0};
	geometry::RigidTransformation rt{rotation, translation};
	EXPECT_TRUE((rt * rt.inverse()).to_transform().isApprox(Eigen::Isometry2d::Identity())); // t t^-1 == identity
	EXPECT_TRUE((rt.inverse() * rt).to_transform().isApprox(Eigen::Isometry2d::Identity())); // t^-1 t == identity
}

// Test if RigidTransformation(Quaternion, Vector3)::inverse is indeed the inverse: t t^-1 == t^-1 t == identity
TEST_F(RigidTransformationTest, inverse3)
{
	Eigen::Quaterniond rotation{Eigen::AngleAxisd{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()}};
	Eigen::Vector3d translation{0.5, -1.0, 2.0};
	geometry::RigidTransformation rt{rotation, translation};
	EXPECT_TRUE((rt * rt.inverse()).to_transform().isApprox(Eigen::Isometry3d::Identity())); // t t^-1 == identity
	EXPECT_TRUE((rt.inverse() * rt).to_transform().isApprox(Eigen::Isometry3d::Identity())); // t^-1 t == identity
}

// Test if RigidTransformation::invert is indeed the in-place equivalent of RigidTransformation::inverse
TEST_F(RigidTransformationTest, invert_inverse)
{
	Eigen::Quaterniond rotation{Eigen::AngleAxisd{30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized()}};
	Eigen::Vector3d translation{0.5, -1.0, 2.0};
	geometry::RigidTransformation rt{rotation, translation};
	geometry::RigidTransformation rt_ = rt;
	rt_.invert();
	EXPECT_TRUE(rt.inverse().to_transform().isApprox(rt_.to_transform()));
}

} // namespace cgogn
