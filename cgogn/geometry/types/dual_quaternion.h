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

#ifndef CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_
#define CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_

#include <cgogn/core/utils/numerics.h>
#include <cgogn/geometry/types/vector_traits.h>
#include <cgogn/geometry/functions/quaternion_operations.h>

namespace cgogn
{

namespace geometry
{

class DualQuaternion
{
public:
	DualQuaternion() = delete;

	[[nodiscard]]
	static inline DualQuaternion from_rotation(const Quaternion& r)
	{
		return DualQuaternion{r, {0, 0, 0}};
	}

	[[nodiscard]]
	static inline DualQuaternion from_translation(const Vec3& t)
	{
		return DualQuaternion{{1, 0, 0, 0}, t};
	}

	[[nodiscard]]
	static inline DualQuaternion from_rt(const Quaternion& r, const Vec3& t)
	{
		return DualQuaternion{r, t};
	}

	[[nodiscard]]
	static inline DualQuaternion from_tr(const Vec3& t, const Quaternion& r)
	{
		return DualQuaternion{t, r};
	}

	[[nodiscard]]
	static inline DualQuaternion identity()
	{
		return DualQuaternion{{1, 0, 0, 0}, {0, 0, 0}};
	}

	[[nodiscard]]
	static inline Scalar dot(const DualQuaternion& a, const DualQuaternion& b)
	{
		return a.dot(b);
	}

	[[nodiscard]]
	inline Quaternion real() const { return r_; }

	[[nodiscard]]
	inline Quaternion dual() const { return d_; }

	[[nodiscard]]
	inline Quaternion rotation() const { return r_; }

	[[nodiscard]]
	inline Vec3 translation() const
	{
		return (d_ * r_.conjugate()).vec() * 2.0;
	}

	[[nodiscard]]
	inline DualQuaternion transform(const DualQuaternion& p) const
	{
		return *this * p * conjugated();
	}

	[[nodiscard]]
	inline Quaternion transform(const Quaternion& r) const
	{
		return transform(from_rotation(r)).rotation();
	}

	[[nodiscard]]
	inline Vec3 transform(const Vec3& t) const
	{
		return transform(from_translation(t)).translation();
	}

	inline void transform_by(const DualQuaternion& q)
	{
		*this = q.transform(*this);
	}

	[[nodiscard]]
	inline Scalar dot(const DualQuaternion& other) const
	{
		return r_.dot(other.r_);
	}

	[[nodiscard]]
	inline Scalar squaredMagnitude() const
	{
		return r_.dot(r_);
	}

	[[nodiscard]]
	inline Scalar magnitude() const
	{
		return std::sqrt(squaredMagnitude());
	}

	inline void normalize()
	{
		Scalar m = magnitude();

		// Not normalizable
		if (cgogn::almost_equal_relative(m, Scalar(0)))
			return;

		r_ *= 1.0 / m;
		d_ *= 1.0 / m;
	}

	[[nodiscard]]
	inline DualQuaternion normalized() const
	{
		DualQuaternion res = *this;
		res.normalize();
		return res;
	}

	inline void conjugate()
	{
		r_ = r_.conjugate();
		d_.w() *= -1.0;
	}

	[[nodiscard]]
	inline DualQuaternion conjugated() const
	{
		return DualQuaternion(r_.conjugate(), d_.conjugate() * -1.0);
	}

	inline bool isApprox(const DualQuaternion& other,
		const Scalar& prec = Eigen::NumTraits<Scalar>::dummy_precision()) const
	{
		return qIsApprox(r_, other.r_, prec) && qIsApprox(d_, other.d_, prec);
	}

	friend inline DualQuaternion operator+(DualQuaternion a, const DualQuaternion& b)
	{
		return DualQuaternion(a.r_ + b.r_, a.d_ + b.d_);
	}

	friend inline DualQuaternion operator*(const Scalar& s, const DualQuaternion& dq)
	{
		return DualQuaternion(s * dq.r_, s * dq.d_);
	}

	friend inline DualQuaternion operator*(const DualQuaternion& dq, const Scalar& s)
	{
		return s * dq;
	}

	friend inline DualQuaternion operator*(const DualQuaternion& a, const DualQuaternion& b)
	{
		return DualQuaternion(a.r_ * b.r_, a.r_ * b.d_ + a.d_ * b.r_);
	}

	friend inline std::ostream& operator<<(std::ostream& os, const DualQuaternion& dq)
	{
		os << dq.r_ << " + (" << dq.d_ << ")e";
		return os;
	}

	inline DualQuaternion& operator+=(const DualQuaternion& other)
	{
		return (*this = *this + other);
	}

	template <class T> // Scalar and DualQuaternion
	inline DualQuaternion& operator*=(const T& other)
	{
		return (*this = *this * other);
	}

private:
	inline explicit DualQuaternion(Quaternion r, Quaternion d) : r_(r), d_(d) {}

	inline explicit DualQuaternion(const Quaternion& r, const Vec3& t) : r_(r.normalized())
	{
		d_ = r * Quaternion{0, 0.5 * t.x(), 0.5 * t.y(), 0.5 * t.z()};
	}

	inline explicit DualQuaternion(const Vec3& t, const Quaternion& r) : r_(r.normalized())
	{
		d_ = Quaternion{0, 0.5 * t.x(), 0.5 * t.y(), 0.5 * t.z()} * r;
	}

	inline static bool qIsApprox(const Quaternion& a, const Quaternion& b, const Scalar& prec)
	{
		// isApprox fails for quaternions close to zero
		// isMuchSmallerThan somehow does not work, so we use squaredNorm instead
		return a.isApprox(b, prec)
				|| a.coeffs().squaredNorm() <= prec * prec && b.coeffs().squaredNorm() <= prec * prec;
	}

private:
	Quaternion r_; // real part (rotation)
	Quaternion d_; // dual part (translation)
};

} // namespace geometry

} // namespace cgogn

#endif // CGOGN_GEOMETRY_TYPES_DUAL_QUATERNION_H_