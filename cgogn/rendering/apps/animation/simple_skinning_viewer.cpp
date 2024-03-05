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

#include <unordered_map>

#include <cgogn/core/types/maps/cmap/cmap2.h>
#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/core/utils/numerics.h>

#include <cgogn/ui/app.h>
#include <cgogn/ui/view.h>

#include <cgogn/geometry/ui_modules/animation_skeleton_controller.h>
#include <cgogn/rendering/ui_modules/animation_skeleton_render.h>
#include <cgogn/geometry/ui_modules/skinning_controller.h>
#include <cgogn/geometry/ui_modules/surface_differential_properties.h>
#include <cgogn/rendering/ui_modules/surface_render.h>
#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/io/ui_modules/fbx_io.h>

using Surface = cgogn::CMap2;
using Skeleton = cgogn::AnimationSkeleton;

template <typename T>
using AttributeM = typename cgogn::mesh_traits<Surface>::Attribute<T>;
using Vertex = typename cgogn::mesh_traits<Surface>::Vertex;
using Edge = typename cgogn::mesh_traits<Surface>::Edge;

template <typename T>
using AttributeS = typename cgogn::mesh_traits<Skeleton>::Attribute<T>;
using Joint = typename cgogn::mesh_traits<Skeleton>::Vertex;
using Bone = typename cgogn::mesh_traits<Skeleton>::Edge;

using Vec3 = cgogn::geometry::Vec3;
using Vec4 = cgogn::geometry::Vec4;
using Vec4i = cgogn::geometry::Vec4i;
using Quaternion = cgogn::geometry::Quaternion;
using Scalar = cgogn::geometry::Scalar;
using RigidTransformation = cgogn::geometry::RigidTransformation<Quaternion, Vec3>;
using cgogn::geometry::DualQuaternion;

using ASC_RT = cgogn::ui::AnimationSkeletonController<std::vector, double, RigidTransformation>;
using ASC_DQ = cgogn::ui::AnimationSkeletonController<std::vector, double, DualQuaternion>;

using namespace cgogn::numerics;

#define DEFAULT_MESH_PATH CGOGN_STR(CGOGN_DATA_PATH) "/meshes/"

template <typename TransformT, typename ASCT>
auto get_transform_attributes(Skeleton& sk, const ASCT& asc)
{
	auto l = cgogn::get_or_add_attribute<TransformT, Bone>(sk, asc.local_transform_attribute_name());
	auto w = cgogn::get_or_add_attribute<TransformT, Bone>(sk, asc.world_transform_attribute_name());
	return std::make_pair(l, w);
}

template <typename TransformT, typename ASCT>
auto setup_transform_attributes_and_get_bb(
		Skeleton& sk,
		const ASCT& asc,
		const AttributeS<cgogn::geometry::KeyframedAnimation<std::vector, double, TransformT>>& anims,
		AttributeS<Vec3>& positions)
{
	auto [l, w] = get_transform_attributes<TransformT>(sk, asc);
	auto res = ASCT::Embedding::compute_animation_bb(sk, anims, *l, *w, positions);
	// Reset to starting position to not mess up binding later
	ASCT::Embedding::compute_everything(ASCT::TimePoint::Start, sk, anims, *l, *w, positions);
	return res;
}

template <typename TransformT, typename ASCT>
auto setup_transform_attributes_and_get_bb(
		Skeleton& sk,
		const ASCT& asc,
		const std::initializer_list<std::shared_ptr<AttributeS<
				cgogn::geometry::KeyframedAnimation<std::vector, double, TransformT>>>>& anims,
		AttributeS<Vec3>& positions)
{
	// We just assume there's two since this function is dedicated to our use case
	auto it = anims.begin();
	auto a = setup_transform_attributes_and_get_bb(sk, asc, **it++, positions);
	auto b = setup_transform_attributes_and_get_bb(sk, asc, **it, positions);
	return std::make_pair<Vec3, Vec3>(a.first.cwiseMin(b.first), a.second.cwiseMax(b.second));
}

auto create_placeholder_skeleton_anim_rt(Skeleton& sk)
{
	using RT = RigidTransformation;
	using KA = cgogn::geometry::KeyframedAnimation<std::vector, double, RT>;

	std::shared_ptr<Skeleton::Attribute<KA>> anim_attr_twist
			= cgogn::add_attribute<KA, Bone>(sk, "placeholder_animation_RT_twist");

	Vec3 t = {1.0, 0.0, 0.0};

	Quaternion r{Eigen::AngleAxisd{M_PI, t}};
	(*anim_attr_twist)[sk.bone_traverser_[0]] = KA{{0.0, RT{}}};
	(*anim_attr_twist)[sk.bone_traverser_[1]] = KA{{0.0, RT{t}}, {1.0, RT{r, t}}};

	Quaternion r_{Eigen::AngleAxisd{M_PI_2, Vec3{0.0, 0.0, 1.0}}};
	std::shared_ptr<Skeleton::Attribute<KA>> anim_attr_bend
			= cgogn::add_attribute<KA, Bone>(sk, "placeholder_animation_RT_bend");

	(*anim_attr_bend)[sk.bone_traverser_[0]] = KA{{0.0, RT{}}};
	(*anim_attr_bend)[sk.bone_traverser_[1]] = KA{{0.0, RT{t}}, {1.0, RT{r_, t}}};

	return std::make_pair(anim_attr_twist, anim_attr_bend);
}

auto create_placeholder_skeleton_anim_dq(Skeleton& sk)
{
	using DQ = DualQuaternion;
	using KA = cgogn::geometry::KeyframedAnimation<std::vector, double, DQ>;

	std::shared_ptr<Skeleton::Attribute<KA>> anim_attr_twist
			= cgogn::add_attribute<KA, Bone>(sk, "placeholder_animation_DQ_twist");

	Vec3 t = {1.0, 0.0, 0.0};

	Quaternion r{Eigen::AngleAxisd{M_PI, t}};
	(*anim_attr_twist)[sk.bone_traverser_[0]] = KA{{0.0, DQ::identity()}};
	(*anim_attr_twist)[sk.bone_traverser_[1]] = KA{{0.0, DQ::from_translation(t)}, {1.0, DQ::from_tr(t, r)}};

	Quaternion r_{Eigen::AngleAxisd{M_PI_2, Vec3{0.0, 0.0, 1.0}}};
	std::shared_ptr<Skeleton::Attribute<KA>> anim_attr_bend
			= cgogn::add_attribute<KA, Bone>(sk, "placeholder_animation_DQ_bend");

	(*anim_attr_bend)[sk.bone_traverser_[0]] = KA{{0.0, DQ::identity()}};
	(*anim_attr_bend)[sk.bone_traverser_[1]] = KA{{0.0, DQ::from_translation(t)}, {1.0, DQ::from_tr(t, r_)}};

	return std::make_pair(anim_attr_twist, anim_attr_bend);
}

auto create_placeholder_skeleton(cgogn::ui::MeshProvider<Skeleton>& mp_as, const ASC_RT& asc_rt, const ASC_DQ& asc_dq)
{
	Skeleton* sk = mp_as.add_mesh("Placeholder skeleton");
	std::shared_ptr<AttributeS<Vec3>> positions = cgogn::get_or_add_attribute<Vec3, Joint>(*sk, "position");

	add_bone(*sk, add_root(*sk));

	auto [anims_rt_t, anims_rt_b] = create_placeholder_skeleton_anim_rt(*sk);
	auto [anims_dq_t, anims_dq_b] = create_placeholder_skeleton_anim_dq(*sk);

	auto bb_rt = setup_transform_attributes_and_get_bb(*sk, asc_rt, {anims_rt_t, anims_rt_b}, *positions);
	auto bb_dq = setup_transform_attributes_and_get_bb(*sk, asc_dq, {anims_dq_t, anims_dq_b}, *positions);

	auto bb = std::make_pair<Vec3, Vec3>(bb_dq.first.cwiseMin(bb_rt.first), bb_dq.second.cwiseMax(bb_rt.second));

	// Double bounding box dimensions to leave more space for skin
	Vec3 d = bb.second - bb.first;
	bb.first -= 0.5 * d;
	bb.second += 0.5 * d;

	return std::make_tuple(sk, positions, bb);
}

std::shared_ptr<AttributeS<Vec3>> create_placeholder_bone_colors(Skeleton& sk)
{
	using S = Vec3::Scalar;
	auto res = cgogn::get_or_add_attribute<Vec3, Bone>(sk, "bone_colors");
	for (int i = 0; i < sk.bone_traverser_.size(); ++i)
		(*res)[sk.bone_traverser_[i]]
				= {static_cast<S>(i < 2 || i > 4), static_cast<S>(i > 0 && i < 4), static_cast<S>(i > 2)};
	return res;
}

auto create_placeholder_weights(Surface& m, const AttributeS<Vec3>& positions)
{
	auto weight_indices = cgogn::get_or_add_attribute<Vec4i, Vertex>(m, "weight_index");
	auto weight_values = cgogn::get_or_add_attribute<Vec4, Vertex>(m, "weight_value");

	cgogn::parallel_foreach_cell(m, [&](Vertex v)
	{
		const auto i = cgogn::index_of(m, v);
		(*weight_indices)[i] = {0, 1, -1, -1};
		(*weight_values)[i] = {1.0 - positions[i].x(), positions[i].x(), 0.0, 0.0};
		return true;
	});

	return std::make_pair(weight_indices, weight_values);
}

int main(int argc, char** argv)
{
	std::string filename;
	bool normalize_surface = true;
	if (argc < 2)
	{
		filename = std::string(DEFAULT_MESH_PATH) + std::string("off/cylinder.off");
		normalize_surface = false;
	}
	else
		filename = std::string(argv[1]);

	cgogn::thread_start();

	cgogn::ui::App app;
	app.set_window_title("Simple graph viewer");
	app.set_window_size(1000, 800);

	auto sp_mp = std::make_shared<cgogn::ui::MeshProvider<Surface>>(app);
	auto sp_mp_as = std::make_shared<cgogn::ui::MeshProvider<Skeleton>>(app);
	auto& mp = *sp_mp;
	auto& mp_as = *sp_mp_as;
	cgogn::ui::FbxIO<Surface> fbx_io(app, sp_mp, sp_mp_as);
	ASC_RT asc_rt(app);
	ASC_DQ asc_dq(app);
	cgogn::ui::SkinningController<Surface, RigidTransformation> skc_rt(app);
	cgogn::ui::SkinningController<Surface, DualQuaternion> skc_dq(app);
	cgogn::ui::AnimationSkeletonRender<RigidTransformation, DualQuaternion> asr(app);
	cgogn::ui::SurfaceRender<Surface> sr(app);
	cgogn::ui::SurfaceDifferentialProperties<Surface> sdp(app);

	app.init_modules();

	cgogn::ui::View* v1 = app.current_view();
	v1->link_module(&mp);
	v1->link_module(&mp_as);
	v1->link_module(&asr);
	v1->link_module(&sr);

	Surface* m = mp.load_surface_from_file(filename, normalize_surface);
	if (!m)
	{
		std::cout << "File could not be loaded" << std::endl;
		return 1;
	}

	std::shared_ptr<AttributeS<Vec3>> vertex_position = cgogn::get_attribute<Vec3, Vertex>(*m, "position");

	auto [weight_index, weight_value] = create_placeholder_weights(*m, *vertex_position);
	auto [sk, joint_position, bb] = create_placeholder_skeleton(mp_as, asc_rt, asc_dq);
	auto bone_color = create_placeholder_bone_colors(*sk);

	skc_rt.set_skeleton(sk);
	skc_rt.set_mesh(m);
	skc_dq.set_skeleton(sk);
	skc_dq.set_mesh(m);

	std::shared_ptr<AttributeS<Scalar>> joint_radius = cgogn::get_attribute<Scalar, Joint>(*sk, "radius");
	mp_as.set_mesh_bb_override(*sk, bb);
	asr.set_joint_position(*v1, *sk, joint_position);
	asr.set_joint_radius(*v1, *sk, joint_radius);
	asr.set_bone_color(*v1, *sk, bone_color);

	std::shared_ptr<AttributeS<Vec3>> vertex_normal = cgogn::get_or_add_attribute<Vec3, Vertex>(*m, "normal");

	mp.set_mesh_bb_override(*m, bb); // need the same bounding box to display in the same place

	sdp.compute_normal(*m, vertex_position.get(), vertex_normal.get());

	sr.set_vertex_position(*v1, *m, vertex_position);
	sr.set_vertex_normal(*v1, *m, vertex_normal);

	return app.launch();
}
