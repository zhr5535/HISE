#pragma once
#ifdef USE_TBB
#include <tbb/tbb.h>
#endif
#include <opencv2/core/traits.hpp>

#include "core/point_search_wrapper.h"
#include "internal/bicubic_spline_interpolation.h"
#include "internal/correlation_kernel_funcs.h"

namespace deflow {
namespace internal {
using interpolation_t = UBSplineInterp6x6_t;
using correlation_t = CorrelateKernels1394_t;

using matrix_t = correlation_match::matrix_t;
using template_t = correlation_match::template_t;
using lutify_t = correlation_match::lut_t;
using brief_options_t = Options::CorrSpec;

namespace ts {
template<typename _Ty> static constexpr auto cexpr_tol = static_cast<_Ty>(1e-3);
}

/// <summary>
/// \brief Return square of given value 'v'.
/// </summary>
template<typename _Ty> inline auto _Sq(_Ty v) noexcept {
	return v * v;
}

template<typename _Pt, typename _Contref = const std::vector<_Pt>&>
auto _Pixel_search(const template_t& _Ref, matrix_t _Tar, _Contref _Pts) 
{
	using value_t = decltype(_Ref.mean);
	const auto _Radius = _Ref.data.rows >> 1;

	const auto _Zncc = [&](const template_t& f, const matrix_t& g) {
		const auto g_mean = (f.mask.empty()?cv::mean(g):cv::mean(g, f.mask))[0];
		auto g_ssd = value_t{ 0 }, x_ssd = value_t{ 0 };
		for (auto r = 0; r < g.rows; ++r) {
			const auto f_row = f.data.ptr<uint8_t>(r);
			const auto g_row = g.ptr<value_t>(r);
			for (auto c = 0; c < g.cols; ++c) {
				const auto mask_v = f.mask.empty() ? true : bool(f.mask.ptr(r)[c]);
				if (mask_v) {
					const auto gv = g_row[c];
					const auto fv = f_row[c];
					const auto gdiff = gv - g_mean;
					g_ssd += gdiff * gdiff;
					const auto fdiff = fv - f.mean;
					x_ssd += fdiff * gdiff;
				}
			}
		}
		const auto deno = std::sqrt(f.ssdv * g_ssd);
		return deno > 0 ? x_ssd / deno : value_t(0);
	};

	std::vector<float_t> _Scores(_Pts.size());
#ifdef USE_TBB
	tbb::parallel_for(size_t{ 0 }, _Pts.size(), [&](auto _Idx) {
		const auto& p = _Pts[_Idx];
		const auto x = int32_t(p.x), y = int32_t(p.y);
		_Scores[_Idx] = _Zncc(_Ref, _Tar({ y - _Radius, y + _Radius+1 }, { x - _Radius, x + _Radius+1 }));
		});
#elif
	size_t _Idx{ 0 };
	std::for_each(std::execution::par, _Pts.begin(), _Pts.end(), [&](auto&& p) {
		const auto x = int32_t(p.x), y = int32_t(p.y);
		_Scores[_Idx++] = _Zncc(_Ref, _Tar({ y - _Radius, y + _Radius + 1 }, { x - _Radius, x + _Radius + 1 }));
		});
#endif
	const auto _Off = std::distance(_Scores.begin(), std::max_element(_Scores.begin(), _Scores.end()));
	return std::make_tuple(_Off, _Off < _Scores.size() ? _Scores[_Off] : 0.);
}

auto _Subpixel_refine(const template_t& f, const lutify_t& g, matrix_t q, brief_options_t opts) {
	using value_t = std::decay_t<decltype(f.mean)>;
	const auto [step_x, step_y, max_iters] = opts;
	value_t f_mean = f.mean;

	decltype(auto) _Def_pars = f.transform;
	auto p = _Def_pars.ptr<value_t>(0);
	auto _Pold = cvMat(_Def_pars);

	// Holds the deformed subset
	auto _G = detail::make_cvmat<value_t>(f.data.rows, f.data.cols);
	auto _Gmat = cvMat(_G);
	auto _Img = cvMat(g.image);
	auto _Lut = cvMat(g.data);
	auto _Mask = cvMat(g.mask);
	auto _Qmat = cvMat(q);

	auto _Fmat = cvMat(f.data);
	auto _Fx = cvMat(f.gx), _Fy = cvMat(f.gy);
	auto _InvH = cvMat(f.inverse_hessian);

	auto dp = detail::make_cvmat<value_t>(6, 1);
	auto _Dp = cvMat(dp);
	auto dptr = dp.ptr<value_t>(0);
	auto p_new = detail::make_cvmat<value_t>(6, 1);
	auto _Pnew = cvMat(p_new);
	auto i = 0;
	auto _Coef = value_t(0);
	for (; i < max_iters; ++i) {
		interpolation_t::Value_One_Affine_Template_64F(&_Img,
			p[0] + f.center.x, p[3] + f.center.y, p[1], p[2], p[4], p[5],
			g.x_min, g.x_max, g.y_min, g.y_max, &_Lut, &_Mask, &_Qmat, &_Gmat);

		auto g_mean = cv::mean(_G)[0], g_ssd = value_t{ 0 };
		correlation_t::CalculatePassdScaleValue(&_Fmat, &_Gmat, &f_mean, &g_mean, &g_ssd);
		correlation_t::Calculate_Newton_Raphson_Get_Inverse_Displacement_Delta(
			&_Fmat, &_Fx, &_Fy, &_Gmat, &f_mean, &g_mean, &g_ssd, &_InvH, &_Dp);
		correlation_t::Calculate_Newton_Raphson_Get_Inverse_Displacement(&_Pold, &_Dp, &_Pnew);

		p_new.copyTo(_Def_pars);

		const auto _Error = _Sq(dptr[0]) + _Sq(step_x * dptr[1]) + _Sq(step_y * dptr[2]) +
			_Sq(dptr[3]) + _Sq(step_x * dptr[4]) + _Sq(step_y * dptr[5]);
		if (std::sqrt(_Error) < ts::cexpr_tol<value_t> || i == max_iters-1) {
			i++;
			interpolation_t::Value_One_Affine_Template_64F(&_Img,
				p[0] + f.center.x, p[3] + f.center.y, p[1], p[2], p[4], p[5],
				g.x_min, g.x_max, g.y_min, g.y_max, &_Lut, &_Mask, &_Qmat, &_Gmat);
			g_mean = cv::mean(_G)[0];
			correlation_t::CalculatePassdScaleValue(&_Fmat, &_Gmat, &f_mean, &g_mean, &g_ssd);
			auto f_ssd = f.ssdv;
			correlation_t::Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient(
				&_Fmat, &_Gmat, &f_mean, &f_ssd, &g_mean, &g_ssd, &_Coef);
			break;
		}
	}
	return std::make_tuple(i, _Coef, _Def_pars);
}

/// <summary>
/// \brief Refine the point transform from f to g with a given initial transformation.
/// </summary>
/// <param name="f"></param>
/// <param name="g"></param>
/// <param name="q"></param>
/// <param name="point_transform">Initial transformation, with [u, ux, uy, v, vx, vy]</param>
/// <param name="opts"></param>
/// <returns>Refined transform</returns>
auto _Subpixel_refine(const template_t& f, const lutify_t& g, matrix_t q, 
	matrix_t point_transform, brief_options_t opts) 
{
	using value_t = std::decay_t<decltype(f.mean)>;
	const auto [step_x, step_y, max_iters] = opts;
	value_t f_mean = f.mean;

	auto _Def_pars = point_transform.cols==1? point_transform.clone(): point_transform.t();
	auto p = _Def_pars.ptr<value_t>(0);
	auto _Pold = cvMat(_Def_pars);

	// Holds the deformed subset
	auto _G = detail::make_cvmat<value_t>(f.data.rows, f.data.cols);
	auto _Gmat = cvMat(_G);
	auto _Img = cvMat(g.image);
	auto _Lut = cvMat(g.data);
	auto _Mask = cvMat(g.mask);
	auto _Qmat = cvMat(q);

	auto _Fmat = cvMat(f.data);
	auto _Fx = cvMat(f.gx), _Fy = cvMat(f.gy);
	auto _InvH = cvMat(f.inverse_hessian);

	auto dp = detail::make_cvmat<value_t>(6, 1);
	auto _Dp = cvMat(dp);
	auto dptr = dp.ptr<value_t>(0);
	auto p_new = detail::make_cvmat<value_t>(6, 1);
	auto _Pnew = cvMat(p_new);
	auto i = 0;
	auto _Coef = value_t(0);
	for (; i < max_iters; ++i) {
		interpolation_t::Value_One_Affine_Template_64F(&_Img,
			p[0] + f.center.x, p[3] + f.center.y, p[1], p[2], p[4], p[5],
			g.x_min, g.x_max, g.y_min, g.y_max, &_Lut, &_Mask, &_Qmat, &_Gmat);

		auto g_mean = cv::mean(_G)[0], g_ssd = value_t{ 0 };
		correlation_t::CalculatePassdScaleValue(&_Fmat, &_Gmat, &f_mean, &g_mean, &g_ssd);
		correlation_t::Calculate_Newton_Raphson_Get_Inverse_Displacement_Delta(
			&_Fmat, &_Fx, &_Fy, &_Gmat, &f_mean, &g_mean, &g_ssd, &_InvH, &_Dp);
		correlation_t::Calculate_Newton_Raphson_Get_Inverse_Displacement(&_Pold, &_Dp, &_Pnew);

		p_new.copyTo(_Def_pars);

		const auto _Error = _Sq(dptr[0]) + _Sq(step_x * dptr[1]) + _Sq(step_y * dptr[2]) +
			_Sq(dptr[3]) + _Sq(step_x * dptr[4]) + _Sq(step_y * dptr[5]);
		if (std::sqrt(_Error) < ts::cexpr_tol<value_t> || i == max_iters - 1) {
			i++;
			interpolation_t::Value_One_Affine_Template_64F(&_Img,
				p[0] + f.center.x, p[3] + f.center.y, p[1], p[2], p[4], p[5],
				g.x_min, g.x_max, g.y_min, g.y_max, &_Lut, &_Mask, &_Qmat, &_Gmat);
			g_mean = cv::mean(_G)[0];
			correlation_t::CalculatePassdScaleValue(&_Fmat, &_Gmat, &f_mean, &g_mean, &g_ssd);
			auto f_ssd = f.ssdv;
			correlation_t::Calculate_Newton_Raphson_Get_ZNCC_Correlation_Coefficient(
				&_Fmat, &_Gmat, &f_mean, &f_ssd, &g_mean, &g_ssd, &_Coef);
			break;
		}
	}
	return std::make_tuple(i, _Coef, _Def_pars);
}

} // namespace internal
} // namespace deflow