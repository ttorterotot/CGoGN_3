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
	// 2D vectors
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector2f{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector2d{}}.Dim == 2);
	// 2D transformations
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Rotation2Df{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Rotation2Dd{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Translation2f{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Translation2d{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Rotation2Df{} * Eigen::Translation2f{}}.Dim == 2);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Rotation2Dd{} * Eigen::Translation2d{}}.Dim == 2);
	// 3D/4D vectors and quaternions
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector3f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector3d{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector4f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Vector4d{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Quaternionf{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Quaterniond{}}.Dim == 3);
	// 3D transformations
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::AngleAxisf{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::AngleAxisd{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Translation3f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Translation3d{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Quaternionf{} * Eigen::Translation3f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Quaterniond{} * Eigen::Translation3d{}}.Dim == 3);
	// Matrices (3D, regular or homogeneous)
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Matrix3f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Matrix3d{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Matrix4f{}}.Dim == 3);
	EXPECT_TRUE(geometry::RigidTransformation{Eigen::Matrix4d{}}.Dim == 3);
}

// Test if scalar is correctly inferred from template resolution
TEST_F(RigidTransformationTest, template_scalar)
{
	// 2D vectors
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector2d{}})::Scalar, double>));
	// 2D transformations
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Rotation2Df{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Rotation2Dd{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Translation2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Translation2d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{
			Eigen::Rotation2Df{} * Eigen::Translation2f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{
			Eigen::Rotation2Dd{} * Eigen::Translation2d{}})::Scalar, double>));
	// 3D/4D vectors and quaternions
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector4f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Vector4d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Quaternionf{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Quaterniond{}})::Scalar, double>));
	// 3D transformations
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::AngleAxisf{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::AngleAxisd{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Translation3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Translation3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Quaternionf{} * Eigen::Translation3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Quaterniond{} * Eigen::Translation3d{}})::Scalar, double>));
	// Matrices (3D, regular or homogeneous)
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Matrix3f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Matrix3d{}})::Scalar, double>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Matrix4f{}})::Scalar, float>));
	EXPECT_TRUE((std::is_same_v<decltype(geometry::RigidTransformation{Eigen::Matrix4d{}})::Scalar, double>));
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
	Eigen::Quaterniond rotation{Eigen::AngleAxisd(30.0, Eigen::Vector3d{1.0, 2.0, 4.0}.normalized())};
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

} // namespace cgogn
