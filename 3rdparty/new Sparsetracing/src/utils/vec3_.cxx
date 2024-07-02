#include "utils/_vec3_.h"
#include <algorithm>
#ifdef __cplusplus
#include <cmath>
#else
#include <math.h>
#endif

template<typename _Ty>
Vec3_<_Ty>::Vec3_() : Vec3_<_Ty>(0)
{
}
template<typename _Ty>
Vec3_<_Ty>::Vec3_(_Ty _val) 
	: x(_val), y(_val), z(_val)
{
}
template<typename _Ty>
Vec3_<_Ty>::Vec3_(_Ty _x, _Ty _y, _Ty _z)
	: x(_x), y(_y), z(_z)
{
}
template<typename _Ty>
Vec3_<_Ty>::Vec3_(const Vec3_& _other)
	: x(_other.x), y(_other.y), z(_other.z)
{
}
template<typename _Ty>
Vec3_<_Ty>::Vec3_(const Vec2_<_Ty>& _v2, _Ty _val)
	: x(_v2.x), y(_v2.y), z(_val)
{
}
//template<typename _Ty>
//Vec3_<_Ty>::Vec3_(const std::array<_Ty, 3>& _arr)
//	:x(_arr[0]), y(_arr[1]), z(_arr[2])
//{
//}
template<typename _Ty>
Vec3_<_Ty>::Vec3_(const std::vector<_Ty>& _arr)
	: x(_arr[0]), y(_arr[1]), z(_arr[2])
{
}
template<typename _Ty>
Vec3_<_Ty>::~Vec3_()
{
}

#pragma region Operators
template<typename _Ty>
Vec3_<_Ty>& Vec3_<_Ty>::operator= (const Vec3_& _other) noexcept //assign
{
	x = _other.x, y = _other.y, z = _other.z;
	return (*this);
}
template<typename _Ty>
 const Vec3_<_Ty> Vec3_<_Ty>::operator+ (const Vec3_& _other) const noexcept //add
{
	return Vec3_(x + _other.x, y + _other.y, z
		+ _other.z);
}
 template<typename _Ty>
 const Vec3_<_Ty> Vec3_<_Ty>::operator- (const Vec3_& _other) const noexcept //subtract
{
	return Vec3_(x - _other.x, y - _other.y, z - _other.z);
}
 template<typename _Ty>
 Vec3_<_Ty> Vec3_<_Ty>::operator* (const _Ty& _fac) const noexcept //scaler multiply
{
	return Vec3_(x *_fac, y *_fac, z *_fac);
}
 template<typename _Ty>
 _Ty Vec3_<_Ty>::operator* (const Vec3_& _other) const noexcept //inner product
{
	return (x * _other.x + y * _other.y + z * _other.z);
}
 template<typename _Ty>
 bool Vec3_<_Ty>::operator== (const Vec3_& _other) const noexcept //strictly equal
{
	return x == _other.x && y == _other.y
		&& z == _other.z;
}
 template<typename _Ty>
 bool Vec3_<_Ty>::operator!= (const Vec3_& _other) const noexcept //non-strictly unequal
{
	_Ty _tolerence = 1.0e-5;
	return (*this - _other).norm_sq() > _tolerence;
}
 template<typename _Ty>
 bool Vec3_<_Ty>::operator< (const Vec3_& _other) const noexcept //less than
{
	return x != _other.x ? x < _other.x :
		y != _other.y ? y < _other.y : z < _other.z;
}
 template<typename _Ty>
 _Ty& Vec3_<_Ty>::operator[](const uint32_t _idx /*_idx = 0, 1, 2*/) noexcept
 {
	 return (&x)[_idx];
 }

#pragma endregion

#pragma region Methods
 template<typename _Ty>
 _Ty Vec3_<_Ty>::norm_1() const
 {
	 if constexpr (std::is_unsigned_v<_Ty>) {
		 return x + y + z;
	 }
	 else {
		 return std::abs(x) + std::abs(y) + std::abs(z);
	 }
 }
 template<typename _Ty>
 _Ty Vec3_<_Ty>::norm_2() const 
 {
	 return static_cast<_Ty>(std::sqrt(double(x*x + y*y + z*z)));
 }
 template<typename _Ty>
 _Ty Vec3_<_Ty>::norm_sq() const 
 {
	 return x*x + y*y + z*z;
 }
 template<typename _Ty>
 _Ty Vec3_<_Ty>::norm_inf() const
 {
	 return std::max({ x, y, z });
 }
 template<typename _Ty>
 Vec3_<_Ty> Vec3_<_Ty>::cross(const Vec3_<_Ty>& _vr) const
 {
	 return Vec3_<_Ty>(y * _vr.z - z * _vr.y,
		 z * _vr.x - x * _vr.z,
		 x * _vr.y - y * _vr.x);
 }
 template<typename _Ty>
 Vec3_<_Ty> Vec3_<_Ty>::normalize() 
 {
	 return *this * (1.0 / norm_2());
 }
 template<typename _Ty>
 Vec3_<_Ty> Vec3_<_Ty>::regulate(const _Ty &_rt)
 {
	 return Vec3_(
		 static_cast<_Ty>(static_cast<int>(x*_rt) / _rt),
		 static_cast<_Ty>(static_cast<int>(y*_rt) / _rt),
		 static_cast<_Ty>(static_cast<int>(z*_rt) / _rt));
 }

 template<typename _Ty>
 Vec3_<int> Vec3_<_Ty>::_to_vec3i() const
 {
	 return Vec3_<int>(static_cast<int>(x),
		 static_cast<int>(y),
		 static_cast<int>(z));
 }
 template<typename _Ty>
 Vec3_<float> Vec3_<_Ty>::_to_vec3f() const
 {
	 return Vec3_<float>(static_cast<float>(x),
		 static_cast<float>(y),
		 static_cast<float>(z));
 }
 template<typename _Ty>
 Vec3_<double> Vec3_<_Ty>::_to_vec3d() const
 {
	 return Vec3_<double>(static_cast<double>(x),
		 static_cast<double>(y),
		 static_cast<double>(z));
 }
 template<typename _Ty>
 std::vector<_Ty> Vec3_<_Ty>::_to_std_vector() const
 {
	 std::vector<_Ty> v3(3);
	 v3[0] = x, v3[1] = y, v3[2] = z;
	 return v3;
 }
#pragma endregion


template class Vec3_<int>;
template class Vec3_<float>;
template class Vec3_<double>;
template class Vec3_<uint32_t>;