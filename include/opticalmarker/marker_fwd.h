/******************************************************************
  This file is part of DGELOM(C) 3D-DeFlow 2021 Application.
  Copyright(C) 2019-2021, Zhilong Su, all rights reserved.
******************************************************************/
#include <array>
#include <tuple>
#pragma once
namespace dgelom
{
	enum class om_marker_tag
	{
		CODED = 0,
		UNCOD = 1,
		GECCM = 2,
		TS = 3
	};
	enum class om_output_type
	{
		IMG = 0,
		PDF = 1
	};

	struct om_page_size
	{
		size_t width, height;
		size_t lr_margin;
		size_t tb_margin;
	};

	struct om_code_sector
	{
		float cx, cy;
		size_t radius;
		int start, span;
		template <typename T>
		inline auto rect() const noexcept
		{
			return T{cx - radius, cy - radius, radius * 2., radius * 2.};
		}
	};

	template <typename _Ty = float>
	struct om_marker_desc
	{
		using index_t = int32_t;
		static constexpr auto invalid_index = -1;

		om_marker_desc() = default;
		om_marker_desc(index_t _id, _Ty _cx, _Ty _cy, _Ty _rx, _Ty _ry, _Ty a = 0, _Ty q = 0) noexcept
		{
			id = _id;
			cx = _cx, cy = _cy;
			rx = _rx, ry = _ry;
			angle = a, quality = q;
		}
		om_marker_desc(const om_marker_desc &other) noexcept
		{
			id = other.id;
			cx = other.cx, cy = other.cy;
			rx = other.rx, ry = other.ry;
			angle = other.angle, quality = other.quality;
		}

		om_marker_desc &operator=(const om_marker_desc &other) noexcept
		{
			id = other.id;
			cx = other.cx, cy = other.cy;
			rx = other.rx, ry = other.ry;
			angle = other.angle, quality = other.quality;
			return (*this);
		}

		index_t id = -1;	// point id, >= 0 for coded markers, -1 for uncoded marker points, -2 for general features
		_Ty cx = 0, cy = 0; // center (cx, cy) of a marker point
		_Ty rx = 1, ry = 1; // radii in x- and y-direction
		_Ty angle = 0;		// with unit of radian
		_Ty quality = 0;	// quality of ellipse fitting (1 - fitting error)

		index_t prev_idx{invalid_index}; // pos index in the previous frame
		index_t next_idx{invalid_index}; // pos index in the next frame

		template <typename _Uy = _Ty>
		inline auto center() const noexcept
		{
			return std::array<_Uy, 2>{_Uy(cx), _Uy(cy)};
		}
		// Return a tuple of {index_t, index_t, std::array<_Ty, 2>}.
		template <typename _Uy = _Ty>
		inline auto brief() const noexcept
		{
			return std::make_tuple(id, next_idx, center<_Uy>());
		}
		// Return a tuple of {-1, next_idx, std::array<_Ty, 2>}.
		template <typename _Uy = _Ty>
		inline auto brief2() const noexcept
		{
			return std::make_tuple(-1, next_idx, center<_Uy>());
		}
	};
	template <typename _Ty = float>
	inline bool is_trivial_marker(const om_marker_desc<_Ty> &m) noexcept
	{
		return m.id == -1;
	}
	template <typename _Ty = float>
	inline bool is_coded_marker(const om_marker_desc<_Ty> &m) noexcept
	{
		return m.id >= 0;
	}
	template <typename _Ty = float>
	inline bool is_general_feature(const om_marker_desc<_Ty> &m) noexcept
	{
		return m.id == -2;
	}
	// Check if the marker "m" exists in the next frame.
	template <typename _Ty = float>
	inline bool has_next(const om_marker_desc<_Ty> &m) noexcept
	{
		return m.next_idx != m.invalid_index;
	}
	// Check if the marker "m" exists in the previous frame.
	template <typename _Ty = float>
	inline bool has_prev(const om_marker_desc<_Ty> &m) noexcept
	{
		return m.prev_idx != m.invalid_index;
	}
	// Check if the markers "m" exists in three consecutive frames.
	template <typename _Ty = float>
	inline bool has_triviews(const om_marker_desc<_Ty> &m) noexcept
	{
		return (m.prev_idx != m.invalid_index && m.next_idx != m.invalid_index);
	}

	template <om_marker_tag _Type>
	class OpticalMarkers
	{
		static_assert(true,
					  "Undefined marker type in OpticalMarkers<_Type>.");
	};
}
