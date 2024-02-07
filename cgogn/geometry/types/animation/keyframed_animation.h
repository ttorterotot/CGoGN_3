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

    /// @brief Sorts keyframes by time, useful if it's unknown whether they were added in order or not.
    /// @param stable whether or not to use a stable sort (preserves the order of equal elements)
    inline void sort(bool stable = true)
    {
        if (stable)
            std::stable_sort(ContainerT<Keyframe>::begin(), ContainerT<Keyframe>::end(), CompareKeyframes);
        else
            std::sort(ContainerT<Keyframe>::begin(), ContainerT<Keyframe>::end(), CompareKeyframes);
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
    /// @return the interpolated transform as `InterpolatedT`
    template <typename InterpolatedT = TransformT,
            typename T = decltype(identity_c<const InterpolatedT&>),
            typename U = decltype(default_lerp<InterpolatedT, TimeT>)>
    [[nodiscard]]
    inline InterpolatedT get_transform(TimeT time,
            T to_interpolation_space = identity_c<InterpolatedT>,
            U interpolate = default_lerp<InterpolatedT, TimeT>) const
    {
        // Not using ::empty so ContainerT doesn't have to implement it
        cgogn_assert(ContainerT<Keyframe>::size() > 0);

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
