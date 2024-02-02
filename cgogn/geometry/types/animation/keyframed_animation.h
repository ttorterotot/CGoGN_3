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

namespace cgogn
{

namespace geometry
{

template <typename TimeT, typename TransformT>
struct AnimationKeyframe
{
    TimeT time_;
    TransformT transform_;

    AnimationKeyframe(TimeT time, TransformT transform)
        : time_(time), transform_(transform) {}
};

template <template <typename> typename ContainerT, typename TimeT, typename TransformT>
using KeyframedAnimation = ContainerT<AnimationKeyframe<TimeT, TransformT>>;

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_KEYFRAMED_ANIMATION_H_
