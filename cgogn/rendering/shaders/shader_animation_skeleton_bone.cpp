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

#include <cgogn/rendering/shaders/shader_animation_skeleton_bone.h>

namespace cgogn
{

namespace rendering
{

ShaderAnimationSkeletonBone* ShaderAnimationSkeletonBone::instance_ = nullptr;

ShaderAnimationSkeletonBone::ShaderAnimationSkeletonBone()
{
	const char* vertex_shader_source = R"(
		#version 330
		in vec3 vertex_position;
		in vec3 clipping_position;
		out vec3 clip_pos_v;
		void main()
		{
			gl_Position =  vec4(vertex_position, 1.0);
			clip_pos_v = clipping_position;
		}
	)";

	const char* geometry_shader_source = R"(
		#version 330
		layout (lines) in;
		layout (triangle_strip, max_vertices = 12) out;

		uniform mat4 projection_matrix;
		uniform mat4 model_view_matrix;
		uniform float base_radius;

		in vec3 clip_pos_v[];
		out float Nz;
		out vec4 posi_clip;

		#define M_PI 3.1415926535897932384626433832795

		void emit_vertex(in vec3 P, in float n, in vec4 pc)
		{
			Nz = n;
			posi_clip = pc;
			gl_Position = projection_matrix * model_view_matrix * vec4(P, 1.0);
			EmitVertex();
		}

		void emit_triangle(in vec3 A, in vec3 B, in vec3 C)
		{
			float n = normalize(cross(B - A, C - A)).z;
			vec4 pc = vec4(0.0);
			emit_vertex(A, n, pc);
			emit_vertex(B, n, pc);
			emit_vertex(C, n, pc);
			EndPrimitive();
		}

		void main()
		{
			vec4 p0 = gl_in[0].gl_Position;
			vec4 p1 = gl_in[1].gl_Position;

			vec3 base_center = p0.xyz / p0.w;
			vec3 tip = p1.xyz / p1.w;

			vec3 X = tip.xyz - base_center.xyz;

			if (dot(X, X) <= 0.0)
				return;

			X = normalize(X);

			vec3 Y = cross(X, vec3(1.0, 0.0, 0.0));

			if (dot(Y, Y) < 1.0)
				Y = cross(X, vec3(0.0, 1.0, 0.0));

			vec3 Z = cross(X, Y);

			float c = cos(2 * M_PI / 3) * base_radius;
			float s = sin(2 * M_PI / 3) * base_radius;
			vec3 base_A = base_center + Y * base_radius;
			vec3 base_B = base_center + Y * c + Z * s;
			vec3 base_C = base_center + Y * c - Z * s;

			emit_triangle(base_A, base_B, base_C);
			emit_triangle(base_A, tip, base_B);
			emit_triangle(base_B, tip, base_C);
			emit_triangle(base_C, tip, base_A);
		}
	)";

	const char* fragment_shader_source = R"(
		#version 330
		uniform vec4 color;
		uniform float lighted;

		in float Nz;
		in vec4 posi_clip;

		out vec3 frag_out;

		void main()
		{
			float lambert = max(1.0 - lighted, Nz); // Nz = dot(N,0,0,1)
			frag_out = color.rgb * lambert;
		}
	)";

	load3_bind(vertex_shader_source, fragment_shader_source, geometry_shader_source, "vertex_position",
			"clipping_position");
	get_uniforms("color", "base_radius", "lighted");
}

void ShaderParamAnimationSkeletonBone::set_uniforms()
{
	shader_->set_uniforms_values(color_, radius_, lighted_);
}

} // namespace rendering

} // namespace cgogn
