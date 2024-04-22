/*******************************************************************************
 * CGoGN                                                                        *
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

#ifndef CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_
#define CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_

#include <boost/core/demangle.hpp>

#include <cgogn/ui/app.h>
#include <cgogn/ui/module.h>

#include <cgogn/core/ui_modules/mesh_provider.h>
#include <cgogn/geometry/algos/animation/animation_skeleton_embedding_helper.h>

namespace cgogn
{

namespace ui
{

using geometry::Vec3;
using geometry::Vec4;
using geometry::Scalar;

template <template <typename> typename ContainerT, typename TimeT, typename TransformT>
class AnimationSkeletonController : public Module
{
public:
	enum class CheckParams
	{
		CheckAndWarnInvalid,
		CheckInvalid,
		AssumeValid,
	};

	enum class PlayMode
	{
		Pause,
		PlayOnce,
		PlayLooping,
	};

	using Embedding = geometry::AnimationSkeletonEmbeddingHelper<ContainerT, TimeT, TransformT>;
	using TimePoint = typename Embedding::TimePoint;

private:
	using MESH = AnimationSkeleton;

	template <typename T>
	using Attribute = AnimationSkeleton::Attribute<T>;

	using Joint = AnimationSkeleton::Joint;
	using Bone = AnimationSkeleton::Bone;

	using AnimationT = geometry::KeyframedAnimation<ContainerT, TimeT, TransformT>;

public:
	AnimationSkeletonController(const App& app,
			const std::string& local_transform_attribute_unique_name = "local_transform_" + get_demangled_transform_name(),
			const std::string& world_transform_attribute_unique_name = "world_transform_" + get_demangled_transform_name()) :
		Module(app, "AnimationSkeletonController (" + std::string{mesh_traits<MESH>::name} + ", " + get_demangled_animation_name() + ")"),
			local_transform_attribute_name_(local_transform_attribute_unique_name),
			world_transform_attribute_name_(world_transform_attribute_unique_name)
	{
	}
	~AnimationSkeletonController()
	{
	}

	/// @brief Changes the linked bone animation attribute.
	/// Does not affect the embedding, see `set_time` and `update_embedding`.
	/// @param attribute the new attribute to use as animations
	/// @param check_params what to check about animation validity (size and sorting)
	void set_animation(const std::shared_ptr<Attribute<AnimationT>>& attribute, CheckParams check_params = CheckParams::CheckAndWarnInvalid)
	{
		selected_animation_ = attribute;

		if (!selected_animation_) // animation unset
		{
			selected_animation_time_extrema_ = {};
			return;
		}

		// Check for empty animations
		if (check_params == CheckParams::CheckAndWarnInvalid && !AnimationT::are_none_empty(
				selected_animation_->begin(), selected_animation_->end()))
			std::cout << "[AnimationSkeletonController::set_animation] Found empty animation, "
					"some bones may be animated improperly" << std::endl;

		// Check if all bones' animations are sorted
		bool all_sorted = check_params == CheckParams::AssumeValid || AnimationT::are_all_sorted(
				selected_animation_->begin(), selected_animation_->end());

		if (!all_sorted && check_params == CheckParams::CheckAndWarnInvalid)
			std::cout << "[AnimationSkeletonController::set_animation] Found unsorted animation, "
					"skeleton may be animated improperly" << std::endl;

		selected_animation_time_extrema_ = AnimationT::compute_keyframe_time_extrema(*selected_animation_, all_sorted);
	}

	/// @brief Changes the linked joint position attribute, and updates it if it's not null and a skeleton is selected.
	/// Assumes that if a skeleton is selected, its world transform attribute also is.
	/// @param attribute the new attribute to use as positions
	void set_joint_position(const std::shared_ptr<Attribute<Vec3>>& attribute)
	{
		selected_joint_position_ = attribute;

		if (selected_skeleton_ && selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	/// @brief Changes the current time of the animation.
	/// Updates positions accordingly if a skeleton and animation are selected.
	/// @param time the new time to set the animation to
	void set_time(TimeT time)
	{
		time_ = time;

		if (selected_skeleton_ && selected_animation_)
			update_embedding();
	}

	/// @brief Changes the current time of the animation, and updates positions accordingly if a skeleton is selected.
	/// @param time_point the new time to set the animation to
	void set_time(TimePoint time_point)
	{
		if (!selected_animation_)
			return;

		if (!selected_animation_time_extrema_)
		{
			set_time(TimeT{});
			return;
		}

		switch (time_point)
		{
		case TimePoint::Start:
			set_time(selected_animation_time_extrema_->first);
			break;
		case TimePoint::End:
			set_time(selected_animation_time_extrema_->second);
			break;
		default:
			cgogn_assert_not_reached("Missing time point case");
		}
	}

	/// @brief Sets the animation, if any, to pause or resume.
	/// @param play_mode see `PlayMode`
	void set_play_mode(PlayMode play_mode)
	{
		play_mode_ = play_mode;
	}

	/// @brief Changes the linked skeleton, and resets attribute selection for it.
	/// @param sk the new skeleton to link to
	void set_skeleton(MESH* sk)
	{
		selected_skeleton_= sk;
		selected_animation_.reset();
		selected_joint_position_ = get_attribute<Vec3, Joint>(*sk, "position"); // nullptr (equiv. to reset) if not found
		selected_bone_local_transform_ = get_or_add_attribute<TransformT, Bone>(*sk, local_transform_attribute_name_);
		selected_bone_world_transform_ = get_or_add_attribute<TransformT, Bone>(*sk, world_transform_attribute_name_);
	}

	/// @return the attribute name for local transforms
	[[nodiscard]]
	const std::string& local_transform_attribute_name() const
	{
		return local_transform_attribute_name_;
	}

	/// @return the attribute name for world transforms
	[[nodiscard]]
	const std::string& world_transform_attribute_name() const
	{
		return world_transform_attribute_name_;
	}

protected:
	void init() override
	{
		mesh_provider_ = static_cast<MeshProvider<MESH>*>(
			app_.module("MeshProvider (" + std::string{mesh_traits<MESH>::name} + ")"));
		last_frame_time_ = App::frame_time_;
	}

	void left_panel() override
	{
		imgui_mesh_selector(mesh_provider_, selected_skeleton_, "Skeleton", [&](MESH& m) { set_skeleton(&m); });

		if (selected_skeleton_)
		{
			imgui_combo_attribute<Bone, AnimationT>(*selected_skeleton_, selected_animation_,
					"Animation", [&](const std::shared_ptr<Attribute<AnimationT>>& attribute){ set_animation(attribute); });
			imgui_combo_attribute<Joint, Vec3>(*selected_skeleton_, selected_joint_position_,
					"Position", [&](const std::shared_ptr<Attribute<Vec3>>& attribute){ set_joint_position(attribute); });
			
			if (selected_animation_)
				show_time_controls();

			ImGui::Separator();

			if (ImGui::Button("Generate bone colors"))
				Embedding::generate_bone_colors(*selected_skeleton_,
						*get_or_add_attribute<Vec3, Bone>(*selected_skeleton_, GENERATED_BONE_COLOR_ATTRIBUTE_NAME));
		}

		advance_play();

		last_frame_time_ = App::frame_time_;
	}

private:
	// Sets the time according to the play mode if an animation is selected.
	void advance_play()
	{
		if (!selected_animation_ || play_mode_ == PlayMode::Pause
				|| !selected_animation_time_extrema_) // no pose
			return;

		const auto& [start_time, end_time] = *selected_animation_time_extrema_;
		cgogn_assert(start_time <= end_time);

		if (start_time == end_time) // single pose
			return;

		if (play_mode_ == PlayMode::PlayOnce && time_ >= end_time)
		{
			set_play_mode(PlayMode::Pause); // shouldn't start playing again if the user rewinds
			return; // end already reached
		}

		TimeT new_time = time_ + static_cast<TimeT>(App::frame_time_ - last_frame_time_);

		// If the animation changed to one that starts later,
		// better to fast-forward to its start than to wait to catch up
		new_time = std::max(new_time, start_time);

		if (play_mode_ == PlayMode::PlayLooping)
			set_time(std::fmod(new_time - start_time, end_time - start_time) + start_time);
		else // PlayMode::PlayOnce
			set_time(std::min(new_time, end_time));
	}

	void show_time_controls()
	{
		if (!selected_animation_time_extrema_) // no pose
		{
				ImGui::TextUnformatted("Empty animation");
				return;
		}

		const auto& [start_time, end_time] = *selected_animation_time_extrema_;
		cgogn_assert(start_time <= end_time);

		if (start_time == end_time) // single pose
		{
			ImGui::LabelText("Time##L", "%.3f", static_cast<float>(time_));

			if (ImGui::Button("Set pose"))
				set_time(TimePoint::Start);

			return;
		}

		float t = static_cast<float>(time_);
		if (ImGui::SliderFloat("Time", &t, start_time, end_time))
			set_time(static_cast<TimeT>(t));

		if (ImGui::Button("<<"))
			set_time(TimePoint::Start);
		show_tooltip_for_ui_above("Rewind");

		ImGui::SameLine();
		show_play_mode_button("><", "Play looping", PlayMode::PlayLooping);
		ImGui::SameLine();

		if (play_mode_ == PlayMode::PlayLooping // avoid flashing at end when looping
				|| time_ < end_time) // end not reached
			show_play_mode_button(">|", "Play once", PlayMode::PlayOnce);
		else if (show_button_and_tooltip("<>", "Play again"))
		{
			set_play_mode(PlayMode::PlayOnce);
			set_time(TimePoint::Start);
		}

		ImGui::SameLine();
		show_play_mode_button("||", "Pause", PlayMode::Pause);
		ImGui::SameLine();

		if (ImGui::Button(">>"))
			set_time(TimePoint::End);
		show_tooltip_for_ui_above("Fast-forward");
	}

	bool show_button_and_tooltip(const char* label, const char* tooltip_text, bool disabled = false)
	{
		bool res = false;

		if (disabled)
			ImGui::BeginDisabled();

		if (ImGui::Button(label))
			res = true;

		if (disabled)
			ImGui::EndDisabled();

		show_tooltip_for_ui_above(tooltip_text);

		return res;
	}

	void show_play_mode_button(const char* label, const char* tooltip_text, PlayMode play_mode)
	{
		if (show_button_and_tooltip(label, tooltip_text, play_mode_ == play_mode))
			set_play_mode(play_mode);
	}

	void show_tooltip_for_ui_above(const char* tooltip_text)
	{
		if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", tooltip_text); // format required to avoid Wformat-security
	}

	// Updates transforms and joint positions.
	// Assumes a skeleton and animation are selected as well as the corresponding transform attributes.
	void update_embedding()
	{
		cgogn_assert(selected_skeleton_ && selected_animation_
				&& selected_bone_world_transform_ && selected_bone_world_transform_);

		Embedding::compute_all_transforms(time_, *selected_skeleton_, *selected_animation_,
				*selected_bone_local_transform_, *selected_bone_world_transform_);

		signal_transform_attribute_changed_no_bb_update(selected_skeleton_, selected_bone_local_transform_.get());
		signal_transform_attribute_changed_no_bb_update(selected_skeleton_, selected_bone_world_transform_.get());

		// It's fine if no position attribute is selected, this should be called again as soon as one is
		if (selected_joint_position_)
			update_joint_positions_and_signal(mesh_provider_, *selected_skeleton_,
					*selected_bone_world_transform_, selected_joint_position_.get());
	}

	// Updates joint positions from the skeleton and transforms,
	// sending an update signal through the mesh provider.
	static void update_joint_positions_and_signal(
			MeshProvider<MESH>* mesh_provider,
			const MESH& as,
			const Attribute<TransformT>& world_transforms,
			Attribute<Vec3>* positions)
	{
		Embedding::compute_joint_positions(as, world_transforms, *positions);

		if (mesh_provider)
			mesh_provider->emit_attribute_changed(as, positions);
	}

	static void signal_transform_attribute_changed_no_bb_update(const MESH* m,
			Attribute<TransformT>* attribute)
	{
		boost::synapse::emit<MeshProvider<MESH>::attribute_changed>(m, attribute);
		boost::synapse::emit<MeshProvider<MESH>::attribute_changed_t<TransformT>>(m, attribute);
	}

	static std::string get_demangled_transform_name()
	{
		return boost::core::demangle(typeid(TransformT).name());
	}

	static std::string get_demangled_animation_name()
	{
		return boost::core::demangle(typeid(AnimationT).name());
	}

private:
	static inline const std::string GENERATED_BONE_COLOR_ATTRIBUTE_NAME = "generated_bone_color";
	PlayMode play_mode_ = PlayMode::Pause;
	decltype(App::frame_time_) last_frame_time_ = 0;
	TimeT time_ = TimeT{};
	MESH* selected_skeleton_ = nullptr;
	std::shared_ptr<Attribute<AnimationT>> selected_animation_ = nullptr;
	std::optional<std::pair<TimeT, TimeT>> selected_animation_time_extrema_ = {};
	std::shared_ptr<Attribute<Vec3>> selected_joint_position_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_local_transform_ = nullptr;
	std::shared_ptr<Attribute<TransformT>> selected_bone_world_transform_ = nullptr;
	std::string local_transform_attribute_name_;
	std::string world_transform_attribute_name_;
	MeshProvider<MESH>* mesh_provider_ = nullptr;
};

} // namespace ui

} // namespace cgogn

#endif // CGOGN_MODULE_ANIMATION_SKELETON_CONTROLLER_H_
