#include "utils/_vec2_.h"
#include <algorithm>
#ifdef __cplusplus
#include <cmath>
#else
#include <math.h>
#endif

template<typename _Ty>
Vec2_<_Ty>::Vec2_() :x(0), y(0)
{
}
template<typename _Ty>
Vec2_<_Ty>::Vec2_(_Ty _val) 
	: x(_val), y(_val)
{
}
template<typename _Ty>
Vec2_<_Ty>::Vec2_(_Ty _x, _Ty _y)
	: x(_x), y(_y)
{
}
template<typename _Ty>
Vec2_<_Ty>::Vec2_(const Vec2_& _other)
	: x(_other.x), y(_other.y)
{
}
//template<typename _Ty>
//Vec2_<_Ty>::Vec2_(const std::array<_Ty, 2>& _arr)
//	:x(_arr[0]), y(_arr[1])
//{
//}
template<typename _Ty>
Vec2_<_Ty>::Vec2_(const std::vector<_Ty>& _arr)
	: x(_arr[0]), y(_arr[1])
{
}
template<typename _Ty>
Vec2_<_Ty>::~Vec2_()
{
}

#pragma region Operators
template<typename _Ty>
const Vec2_<_Ty> Vec2_<_Ty>::operator= (const Vec2_& _other) //assign
{
	return Vec2_(x = _other.x, y = _other.y);
}
template<typename _Ty>
 const Vec2_<_Ty> Vec2_<_Ty>::operator+ (const Vec2_& _other) const //add
{
	return Vec2_(x + _other.x, y + _other.y);
}

 template<typename _Ty>
 const Vec2_<_Ty>& Vec2_<_Ty>::operator+= (const Vec2_& _other) const //add
 {
	 x += _other.x, y += _other.y;
	 return (*this);
 }

template<typename _Ty>
const Vec2_<_Ty> Vec2_<_Ty>::operator+ (const _Ty _val) const //add
{
	return Vec2_(x + _val, y + _val);
}
template<typename _Ty>
const Vec2_<_Ty>& Vec2_<_Ty>::operator+= (const _Ty& _val) const //add
{
	x += _val, y += _val;

	return (*this);
}
 template<typename _Ty>
 const Vec2_<_Ty> Vec2_<_Ty>::operator- (const Vec2_& _other) const //subtract
{
	return Vec2_(x - _other.x, y - _other.y);
}
 template<typename _Ty>
 const Vec2_<_Ty> Vec2_<_Ty>::operator- (const _Ty _val) const //add
 {
	 return Vec2_(x - _val, y - _val);
 }
 template<typename _Ty>
 Vec2_<_Ty> Vec2_<_Ty>::operator* (const _Ty& _fac) //scaler multiply
{
	return Vec2_(x *_fac, y *_fac);
}
 template<typename _Ty>
 _Ty Vec2_<_Ty>::operator* (const Vec2_& _other) //inner product
{
	return (x * _other.x + y * _other.y);
}
 template<typename _Ty>
 bool Vec2_<_Ty>::operator== (const Vec2_& _other) const //strictly equal
{
	return x == _other.x && y == _other.y;
}
 template<typename _Ty>
 bool Vec2_<_Ty>::operator!= (const Vec2_& _other) const //non-strictly unequal
{
	_Ty _tolerence = 1.0e-6;
	return (*this - _other).norm_sq() > _tolerence;
}
 template<typename _Ty>
 bool Vec2_<_Ty>::operator< (const Vec2_& _other) const //less than
{
	return x < _other.x && y < _other.y;
}
 template<typename _Ty>
 bool Vec2_<_Ty>::operator<(_Ty _val) const
 {
	 return x < _val&& y < _val;
 }
 template<typename _Ty>
 _Ty& Vec2_<_Ty>::operator[](const int _idx /*_idx = 0, 1*/)
 {
	 switch (_idx)
	 {
	 case 0: return (x); break;
	 default: return (y); break;
	 }
 }
 template<typename _Ty>
 _Ty& Vec2_<_Ty>::operator[](const size_t _idx /*_idx = 0, 1*/)
 {
	 switch (_idx)
	 {
	 case 0: return (x); break;
	 default: return (y); break;
	 }
 }
#pragma endregion

#pragma region interface for OCV
#ifdef USE_OCV
 template<typename _Ty>
 Vec2_<_Ty>::Vec2_(const cv::Point_<_Ty>& _other)
	 : x(_other.x), y(_other.y)
 {
 }
 template<typename _Ty>
 const Vec2_<_Ty> Vec2_<_Ty>::operator= (const cv::Point_<_Ty>& _other)
 {
	 return Vec2_(_other.x, _other.y);
 }
#endif
#pragma endregion


#pragma region Methods
 template<typename _Ty>
 _Ty Vec2_<_Ty>::norm_1() const
 {
	 if constexpr (std::is_unsigned_v<_Ty>) {
		 return x + y;
	 }
	 else {
		 return std::abs(x) + std::abs(y);
	 }
 }
 template<typename _Ty>
 _Ty Vec2_<_Ty>::norm_2() const 
 {
	 return static_cast<_Ty>(std::sqrt(double(x*x + y*y)));
 }
 template<typename _Ty>
 _Ty Vec2_<_Ty>::norm_sq() const 
 {
	 return x*x + y*y;
 }
 template<typename _Ty>
 _Ty Vec2_<_Ty>::norm_inf() const
 {
	 return std::max(x, y);
 }
 template<typename _Ty>
 _Ty Vec2_<_Ty>::sum() const
 {
	 return (x + y);
 }
 template<typename _Ty>
 Vec2_<_Ty> Vec2_<_Ty>::cross(const Vec2_<_Ty>& _vr) const
 {
	 _Ty w = x * _vr.y - y * _vr.x;
	 _Ty x_ = (y - _vr.y) * 1.0 / w;
	 _Ty y_ = (_vr.x - x) * 1.0 / w;
	 return Vec2_<_Ty>(static_cast<_Ty>(x_), 
		 static_cast<_Ty>(y_));
 }
 template<typename _Ty>
 Vec2_<_Ty> Vec2_<_Ty>::normalize() 
 {
	 return *this * (1.0 / norm_2());
 }
 template<typename _Ty>
 Vec2_<_Ty> Vec2_<_Ty>::regulate(const _Ty _rt)
 {
	 return Vec2_(
		 static_cast<_Ty>(static_cast<int>(x*_rt) / _rt),
		 static_cast<_Ty>(static_cast<int>(y*_rt) / _rt));
 }

 template<typename _Ty>
 Vec2_<int> Vec2_<_Ty>::_to_vec2i() const
 {
	 return Vec2_<int>(static_cast<int>(x),
		 static_cast<int>(y));
 }
 template<typename _Ty>
 Vec2_<float> Vec2_<_Ty>::_to_vec2f() const
 {
	 return Vec2_<float>(static_cast<float>(x),
		 static_cast<float>(y));
 }
 template<typename _Ty>
 Vec2_<double> Vec2_<_Ty>::_to_vec2d() const
 {
	 return Vec2_<double>(static_cast<double>(x),
		 static_cast<double>(y));
 }
 template<typename _Ty>
 std::vector<_Ty> Vec2_<_Ty>::_to_std_vector() const
 {
	 std::vector<_Ty> v2(2);
	 v2[0] = x, v2[1] = y;
	 return v2;
 }
#pragma endregion


template class Vec2_<int>;
template class Vec2_<float>;
template class Vec2_<double>;
template class Vec2_<uint32_t>;