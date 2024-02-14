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

#ifndef CGOGN_GEOMETRY_TYPES_KEYFRAMED_ANIMATION_H_
#define CGOGN_GEOMETRY_TYPES_KEYFRAMED_ANIMATION_H_

#include <set>

#include <cgogn/core/functions/identity.h>
#include <cgogn/core/utils/type_traits.h>
#include <cgogn/core/utils/assert.h>

namespace cgogn
{

namespace geometry
{

template <typename TimeT, typename TransformT>
struct AnimationKeyframe
{
	using Time = TimeT;
	using Transform = TransformT;

	TimeT time_;
	TransformT transform_;

	AnimationKeyframe(TimeT time, TransformT transform)
		: time_(time), transform_(transform) {}
};

template <template <typename> typename ContainerT, typename TimeT, typename TransformT>
class KeyframedAnimation : public ContainerT<AnimationKeyframe<TimeT, TransformT>>
{
public:
	using Keyframe = AnimationKeyframe<TimeT, TransformT>;
	static_assert(!std::is_integral_v<TimeT>, "Time type should not be integral");

private:

	static constexpr const auto CompareKeyframes = [](const Keyframe& a, const Keyframe& b)
	{
		return a.time_ < b.time_;
	};

	template <typename T, typename S>
	static inline T default_lerp(const T& a, const T& b, const S& t)
	{
		return (1.0 - t) * a + t * b;
	}

public:

	using ContainerT<Keyframe>::ContainerT; // inherit container's constructors

	/// @brief Checks whether each animation in the container has at least one keyframe.
	/// @param begin the iterator to start checking from
	/// @param end the past-the-end iterator to stop checking at
	/// @return whether or not each animation has at least one keyframe
	template <typename It>
	static bool are_none_empty(It begin, It end)
	{
		return std::find_if(begin, end, [](const KeyframedAnimation& anim){ return anim.size() == 0; }) == end;
	}

	/// @brief Checks whether each animation in the container is sorted by time.
	/// Does not check if there's at least one keyframe per animation, for that see `are_none_empty`.
	/// @param begin the iterator to start checking from
	/// @param end the past-the-end iterator to stop checking at
	/// @param warn whether or not to print warnings in the standard output if the animation is found not to be sorted
	/// @return whether or not the animation is sorted
	template <typename It>
	static bool are_all_sorted(It begin, It end)
	{
		return std::find_if_not(begin, end, [](const KeyframedAnimation& anim){ return anim.is_sorted(); }) == end;
	}

	/// @brief Computes the first and last keyframe's times across all animations.
	/// If no animation has any keyframe, then `start_time > end_time`.
	/// @param anims the container containing the animations
	/// @param start_time the variable to update with the start time
	/// @param end_time the variable to update with the end time
	/// @param all_sorted whether or not all animations can be expected to be sorted
	template <template <typename> typename ContainerOther>
	static void compute_keyframe_time_extrema(const ContainerOther<KeyframedAnimation>& anims,
			TimeT& start_time, TimeT& end_time, bool all_sorted = false)
	{
		start_time = std::numeric_limits<TimeT>::max();
		end_time = std::numeric_limits<TimeT>::lowest();

		if (all_sorted)
		{
			for (const auto& anim : anims)
			{
				if (anim.size() > 0)
				{
					start_time = std::min(start_time, anim[0].time_);
					end_time = std::max(end_time, anim[anim.size() - 1].time_);
				}
			}
		}
		else
		{
			for (const auto& anim : anims)
			{
				for (const auto& keyframe : anim)
				{
					start_time = std::min(start_time, keyframe.time_);
					end_time = std::max(end_time, keyframe.time_);
				}
			}
		}
	}

	/// @brief Computes the (ordered) set of keyframes' times across all animations.
	/// @param anims the animation container to get keyframes from
	/// @return the set of times of the animation
	template <template <typename> typename ContainerOther>
	static std::set<TimeT> get_unique_keyframe_times(const ContainerOther<KeyframedAnimation>& anims)
	{
		std::set<TimeT> res;

		for (const auto& anim : anims)
			for (const auto& keyframe : anim)
				res.insert(keyframe.time_);

		return res;
	}

	/// @brief Computes the (ordered) set of keyframes' times across all animations.
	/// @param anims the animation container to get keyframes from
	/// @param prec the minimum distance from existing times that each new one has to be from to be added
	/// @return the set of times of the animation
	template <template <typename> typename ContainerOther>
	static std::set<TimeT> get_unique_keyframe_times(const ContainerOther<KeyframedAnimation>& anims, TimeT prec)
	{
		std::set<TimeT> res{[&prec](const TimeT& a, const TimeT& b){ return a + prec < b; }};

		for (const auto& anim : anims)
			for (const auto& keyframe : anim)
				res.insert(keyframe.time_);

		return res;
	}

	/// @brief Evaluates if the animation's keyframes are sorted in the container.
	/// @return whether or not the keyframes are sorted
	[[nodiscard]]
	inline bool is_sorted() const
	{
		return std::is_sorted(ContainerT<Keyframe>::cbegin(), ContainerT<Keyframe>::cend(), CompareKeyframes);
	}

	/// @brief Sorts keyframes by time, useful if it's unknown whether they were added in order or not.
	/// @param stable whether or not to use a stable sort (preserves the order of equal elements)
	inline void sort(bool stable = true)
	{
		if (stable)
			std::stable_sort(ContainerT<Keyframe>::begin(), ContainerT<Keyframe>::end(), CompareKeyframes);
		else
			std::sort(ContainerT<Keyframe>::begin(), ContainerT<Keyframe>::end(), CompareKeyframes);
	}

	/// @brief Finds the earliest and latest keyframes.
	/// If the keyframes can be assumed to be sorted, use `operator[]` on `0` and `size() - 1` instead.
	/// @return a pair containing the most extreme keyframes
	template <typename It>
	[[nodiscard]]
	inline std::pair<It, It> minmax_element() const
	{
		return std::minmax_element(ContainerT<Keyframe>::cbegin(), ContainerT<Keyframe>::cend(), CompareKeyframes);
	}

	/// @brief Interpolates a transform between both neighboring keyframes.
	/// Requires keyframes to be sorted, see sort().
	/// Converts transforms to interpolate - using `to_interpolation_space` -
	/// then either returns the result - constant extrapolation - outside keyframes,
	/// or interpolates between both neighboring transforms - using `interpolate` - between keyframes.
	/// The possible conversion of the return value from interpolation space is left to the caller.
	/// @param time the time value to interpolate for
	/// @param to_interpolation_space a mapping function from storage to interpolation space in the form
	///                               `(const TransformT&) -> InterpolatedT`
	/// @param interpolate an interpolation function in the form
	///                    `(const InterpolatedT&, const InterpolatedT&, const TimeT&) -> InterpolatedT`
	/// @param default_value the return value if the animation is empty
	/// @return the interpolated transform as `InterpolatedT`
	template <typename InterpolatedT = TransformT,
			typename T = decltype(identity_c<const InterpolatedT&>),
			typename U = decltype(default_lerp<InterpolatedT, TimeT>)>
	[[nodiscard]]
	inline InterpolatedT get_transform(TimeT time,
			T to_interpolation_space = identity_c<InterpolatedT>,
			U interpolate = default_lerp<InterpolatedT, TimeT>,
			InterpolatedT default_value = InterpolatedT{}) const
	{
		// Not using ::empty so ContainerT doesn't have to implement it
		if (ContainerT<Keyframe>::size() <= 0)
			return default_value;

		auto it = std::find_if(ContainerT<Keyframe>::cbegin(), ContainerT<Keyframe>::cend(),
				[&time](const Keyframe& k){ return k.time_ > time; });

		if (it == ContainerT<Keyframe>::cbegin())
			return to_interpolation_space((*this)[0].transform_);

		if (it == ContainerT<Keyframe>::cend())
			return to_interpolation_space((*this)[ContainerT<Keyframe>::size() - 1].transform_);

		const Keyframe& before = *(it - 1);
		const Keyframe& after = *it;

		return interpolate(
				to_interpolation_space(before.transform_),
				to_interpolation_space(after.transform_),
				(time - before.time_) / (after.time_ - before.time_));
	}
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_KEYFRAMED_ANIMATION_H_
