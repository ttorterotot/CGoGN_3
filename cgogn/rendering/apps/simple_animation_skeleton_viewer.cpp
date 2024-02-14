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

#include <cgogn/core/types/animation/animation_skeleton.h>
#include <cgogn/geometry/types/vector_traits.h>

#include <cgogn/ui/app.h>
#include <cgogn/ui/view.h>

#include <cgogn/geometry/ui_modules/animation_skeleton_controller.h>
#include <cgogn/rendering/ui_modules/animation_skeleton_render.h>
#include <cgogn/core/ui_modules/mesh_provider.h>

using Mesh = cgogn::AnimationSkeleton;

template <typename T>
using Attribute = typename cgogn::mesh_traits<Mesh>::Attribute<T>;
using Vertex = typename cgogn::mesh_traits<Mesh>::Vertex;
using Edge = typename cgogn::mesh_traits<Mesh>::Edge;

using Vec3 = cgogn::geometry::Vec3;
using Quaternion = cgogn::geometry::Quaternion;
using Scalar = cgogn::geometry::Scalar;
using RigidTransformation = cgogn::geometry::RigidTransformation<Quaternion, Vec3>;
using cgogn::geometry::DualQuaternion;

using ASC_RT = cgogn::ui::AnimationSkeletonController<std::vector, double, RigidTransformation>;
using ASC_DQ = cgogn::ui::AnimationSkeletonController<std::vector, double, DualQuaternion>;

template <typename TransformT, typename ASCT>
auto setup_transform_attributes_and_get_bb(
		Mesh* m,
		const ASCT& asc,
		const Attribute<cgogn::geometry::KeyframedAnimation<std::vector, double, TransformT>>& anims,
		Attribute<Vec3>& positions)
{
	auto rt_l = cgogn::add_attribute<TransformT, Edge>(*m, asc.local_transform_attribute_name());
	auto rt_w = cgogn::add_attribute<TransformT, Edge>(*m, asc.world_transform_attribute_name());
	return ASCT::Embedding::compute_animation_bb(*m, anims, *rt_l, *rt_w, positions);
}

auto create_placeholder_skeleton_anim_rt(Mesh* m)
{
	using RT = RigidTransformation;
	using KA = cgogn::geometry::KeyframedAnimation<std::vector, double, RT>;

	std::shared_ptr<Mesh::Attribute<KA>> anim_attr
			= cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation_RT");
	KA anim;
	anim.emplace_back(0.0, RT{Vec3{0, 0, 0}});
	anim.emplace_back(1.0, RT{Vec3{1, 0, 0}});
	(*anim_attr)[m->bone_traverser_[0]] = std::move(anim);
	(*anim_attr)[m->bone_traverser_[1]] = KA{{0.0, RT{Vec3{0, 1, 0}}}};
	(*anim_attr)[m->bone_traverser_[2]] = KA{{0.0, RT{Vec3{0, 0, 1}}}};
	(*anim_attr)[m->bone_traverser_[3]] = KA{{0.0, RT{Vec3{0, 1, 0}}}};

	Mesh::Attribute<KA>& anim_empty
			= *cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation_RT_empty");
	for (int i = 0; i < 4; ++i)
		anim_empty[m->bone_traverser_[i]] = KA();

	Mesh::Attribute<KA>& anim_partial
			= *cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation_RT_partial");
	for (int i = 0; i < 4; i += 2) // some keyframes missing
		anim_partial[m->bone_traverser_[i]] = KA{{16.0, RT{Vec3{1, 0, 0}}}};

	Mesh::Attribute<KA>& anim_single
			= *cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation_RT_single");
	for (int i = 0; i < 4; ++i)
		anim_single[m->bone_traverser_[i]]
				= KA{{-64.0, RT{Vec3{static_cast<Vec3::Scalar>(i), 0, 0}}}};

	return anim_attr;
}

auto create_placeholder_skeleton_anim_dq(Mesh* m)
{
	using DQ = DualQuaternion;
	using KA = cgogn::geometry::KeyframedAnimation<std::vector, double, DQ>;

	std::shared_ptr<Mesh::Attribute<KA>> anim_attr
			= cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation_DQ");

	for (int i = 0; i < 4; ++i)
	{
		KA anim;
		anim.emplace_back(0.0, DQ::from_translation({0, 0.25, 1}));
		anim.emplace_back(1.0, DQ::from_rt(
			Quaternion{Eigen::AngleAxisd(0.25 * M_PI, Vec3::UnitZ())}, {0, 0.25, 1}));
		(*anim_attr)[m->bone_traverser_[i]] = std::move(anim);
	}

	return anim_attr;
}

auto create_placeholder_skeleton(cgogn::ui::MeshProvider<Mesh>& mp, const ASC_RT& asc_rt, const ASC_DQ& asc_dq)
{
	Mesh* m = mp.add_mesh("Placeholder");
	std::shared_ptr<Attribute<Vec3>> positions = cgogn::get_or_add_attribute<Vec3, Vertex>(*m, "position");

	Mesh::Bone b;
	for (int i = 0; i < 4; ++i)
		i == 0 ? (b = add_root(*m)) : (b = add_bone(*m, b));

	auto anims_rt = create_placeholder_skeleton_anim_rt(m);
	auto anims_dq = create_placeholder_skeleton_anim_dq(m);

	auto bb_dq = setup_transform_attributes_and_get_bb(m, asc_dq, *anims_dq, *positions);
	auto bb_rt = setup_transform_attributes_and_get_bb(m, asc_rt, *anims_rt, *positions);

	auto bb = std::make_pair<Vec3, Vec3>(bb_dq.first.cwiseMin(bb_rt.first), bb_dq.second.cwiseMax(bb_rt.second));

	return std::make_tuple(m, positions, bb);
}

int main(int argc, char** argv)
{
	cgogn::thread_start();

	cgogn::ui::App app;
	app.set_window_title("Simple graph viewer");
	app.set_window_size(1000, 800);

	cgogn::ui::MeshProvider<Mesh> mp(app);
	ASC_RT asc_rt(app);
	ASC_DQ asc_dq(app);
	cgogn::ui::AnimationSkeletonRender asr(app);

	app.init_modules();

	cgogn::ui::View* v1 = app.current_view();
	v1->link_module(&mp);
	v1->link_module(&asr);

	auto [m, joint_position, bb] = create_placeholder_skeleton(mp, asc_rt, asc_dq);

	std::shared_ptr<Attribute<Scalar>> joint_radius = cgogn::get_attribute<Scalar, Vertex>(*m, "radius");
	mp.set_mesh_bb_override(*m, bb);
	asr.set_joint_position(*v1, *m, joint_position);
	asr.set_joint_radius(*v1, *m, joint_radius);

	return app.launch();
}
