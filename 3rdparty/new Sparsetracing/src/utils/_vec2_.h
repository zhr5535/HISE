#pragma once

#include "sparse/__defs_ports__.hpp"

#include <vector>
#include <array>
#ifdef DEFLOW_USE_CV
#include <opencv2/core/core.hpp>
#endif

///<summary>
//<brief> intro: generic 2-vector class with fields {x, y} </brief>
//<para> author: Su Deglom on 19.Dec.2016 </para>
///</summary>
template<typename _Ty> 
class DEFLOW_ALG Vec2_
{
public:
	Vec2_();
	Vec2_(_Ty _val);
	Vec2_(_Ty _x, _Ty _y);
	Vec2_(const Vec2_& _other);
	//Vec2_(const array<_Ty, 2>& _arr);
	Vec2_(const std::vector<_Ty>& _arr);

	~Vec2_();

	/*operators*/
	const Vec2_ operator= (const Vec2_& _other);
	const Vec2_ operator+ (const Vec2_& _other) const;
	const Vec2_ operator+ (const _Ty _val) const;
	const Vec2_ operator- (const Vec2_& _other) const;
	const Vec2_ operator- (const _Ty _val) const;
	const Vec2_& operator+= (const Vec2_& _other) const;
	const Vec2_& operator+= (const _Ty& _val) const;
	Vec2_ operator* (const _Ty& _fac);
	_Ty operator* (const Vec2_& _other);
	bool operator== (const Vec2_& _other) const;
	bool operator!= (const Vec2_& _other) const;
	bool operator< (const Vec2_& _other) const;
	bool operator< (_Ty _val) const;
	_Ty& operator[](const int _idx);
	_Ty& operator[](const size_t _idx);

	/*interface methods*/
#ifdef DEFLOW_USE_CV
	Vec2_(const cv::Point_<_Ty>& _other);
	const Vec2_ operator=(const cv::Point_<_Ty>& _other);
#endif

	/*methods*/
	//1-norm, namely the sum of the absolute values of all entries
	_Ty norm_1() const;
	//2-norm, namely vector mold
	_Ty norm_2() const;
	// square norm
	_Ty norm_sq() const;
	// infinite norm, namely the absolute of the maximum
	_Ty norm_inf() const;
	// sum of to components: return (x+y)
	_Ty sum() const;
	// cross
	Vec2_ cross(const Vec2_<_Ty>& _vr) const;
	// normalization
	Vec2_ normalize();
	// truncate components to obtain required significant digit
	Vec2_ regulate(const _Ty _rt);

	template<typename _Uy>
	inline Vec2_<_Uy> to() const noexcept {
		return { _Uy(x), _Uy(y) };
	}

	/*private methods*/
private:
	Vec2_<int> _to_vec2i() const;
	Vec2_<float> _to_vec2f() const;
	Vec2_<double> _to_vec2d() const;
	std::vector<_Ty> _to_std_vector() const;

	/*fields*/
public:
	mutable _Ty x, y;
};