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
using Scalar = cgogn::geometry::Scalar;
using RigidTransformation = cgogn::geometry::RigidTransformation<cgogn::geometry::Quaternion, Vec3>;

Mesh* create_placeholder_skeleton(cgogn::ui::MeshProvider<Mesh>& mp)
{
	using RT = RigidTransformation;
	using KA = cgogn::geometry::KeyframedAnimation<std::vector, double, RT>;

	Mesh* m = mp.add_mesh("Placeholder");
	add_bone(*m, add_root(*m));

	std::shared_ptr<Mesh::Attribute<KA>> anim_attr
			= cgogn::add_attribute<KA, Edge>(*m, "placeholder_animation");
	KA anim;
	anim.emplace_back(0.0, RT{Vec3{0, 0, 0}});
	anim.emplace_back(1.0, RT{Vec3{1, 0, 0}});
	(*anim_attr)[m->bone_traverser_[0]] = std::move(anim);
	(*anim_attr)[m->bone_traverser_[1]] = KA{{0.0, RT{Vec3{0, 1, 0}}}};

	return m;
}

void set_placeholder_positions(Attribute<Vec3>& positions)
{
	positions[0] = {-4, -4, -4};
	positions[1] = {4, 4, 4};
}

int main(int argc, char** argv)
{
	cgogn::thread_start();

	cgogn::ui::App app;
	app.set_window_title("Simple graph viewer");
	app.set_window_size(1000, 800);

	cgogn::ui::MeshProvider<Mesh> mp(app);
	cgogn::ui::AnimationSkeletonController<std::vector, double, RigidTransformation> asc_rt(app);
	cgogn::ui::AnimationSkeletonController<std::vector, double, cgogn::geometry::DualQuaternion> asc_dq(app);
	cgogn::ui::AnimationSkeletonRender asr(app);

	app.init_modules();

	cgogn::ui::View* v1 = app.current_view();
	v1->link_module(&mp);
	v1->link_module(&asr);

	Mesh* m = create_placeholder_skeleton(mp);

	std::shared_ptr<Attribute<Vec3>> joint_position = cgogn::get_or_add_attribute<Vec3, Vertex>(*m, "position");
	std::shared_ptr<Attribute<Scalar>> joint_radius = cgogn::get_attribute<Scalar, Vertex>(*m, "radius");
	set_placeholder_positions(*joint_position);
	mp.set_mesh_bb_vertex_position(*m, joint_position);
	asr.set_joint_position(*v1, *m, joint_position);
	asr.set_joint_radius(*v1, *m, joint_radius);

	return app.launch();
}
