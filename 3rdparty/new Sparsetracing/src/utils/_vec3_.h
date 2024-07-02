#pragma once

#include "_vec2_.h"
#include <vector>
#include <array>

///<summary>
///<brief> Generic 3-vector class with fields {x, y, z} </brief>
///<para> Coded by Deglom Su on Dec/19/2016, Modified on Dec/19/2016 </para>
///</summary>
template<typename _Ty> 
class DEFLOW_ALG Vec3_
{
public:
	Vec3_();
	Vec3_(_Ty _val);
	Vec3_(_Ty _x, _Ty _y, _Ty _z);
	Vec3_(const Vec3_& _other);
	Vec3_(const Vec2_<_Ty>& _v2, _Ty _val = 1);
	Vec3_(const std::vector<_Ty>& _arr);

	~Vec3_();

	/*operators*/
public:
	Vec3_& operator= (const Vec3_& _other) noexcept;
	const Vec3_ operator+ (const Vec3_& _other) const noexcept;
	const Vec3_ operator- (const Vec3_& _other) const noexcept;
	Vec3_ operator* (const _Ty& _val) const noexcept;
	_Ty  operator* (const Vec3_& _other) const noexcept;
	bool operator== (const Vec3_& _other) const noexcept;
	bool operator!= (const Vec3_& _other) const noexcept;
	bool operator< (const Vec3_& _other) const noexcept;
	_Ty& operator[](const uint32_t _idx) noexcept;

	inline const _Ty& operator[](const uint32_t _idx) const noexcept {
		return (*this)[_idx];
	}
	inline Vec3_ operator/ (const _Ty& _val) const noexcept {
		return this->operator*(1 / _val);
	}

	/*methods*/
public:
	//1-norm, namely the sum of the absolute values of all entries
	_Ty norm_1() const;
	//2-norm, namely vector mold
	_Ty norm_2() const;
	// square norm
	_Ty norm_sq() const;
	// infinite norm, namely the absolute of the maximum
	_Ty norm_inf() const;
	// cross
	Vec3_ cross(const Vec3_<_Ty>& _vr) const;
	// normalization
	Vec3_ normalize();
	// truncate components to obtain required significant digit
	Vec3_ regulate(const _Ty &_rt);

	/*private methods*/
private:
	Vec3_<int> _to_vec3i() const;
	Vec3_<float> _to_vec3f() const;
	Vec3_<double> _to_vec3d() const;
	std::vector<_Ty> _to_std_vector() const;

	/*fields*/
public:
	_Ty x, y, z;
};