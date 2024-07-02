#pragma once

#include <opencv2/core/core_c.h>
#include <opencv2/calib3d.hpp>

#include "sparse/core/deflow_options.h"
#include "utils/functions.h"
#include "misc/helpmethod.h"

namespace deflow {
DEFLOW_ALG void Options::make_roi(const QList<QList<QPointF>>& polys)
{
	auto orig_mask = detail::make_cvmat<uint8_t>(imrows, imcols);
	auto _Mask = cvMat(orig_mask);
	helper::_Poly_to_mat(polys, &_Mask, &roi_lt_x, &roi_lt_y, &roi_width, &roi_height);
	int _Meshed_mask_cols{ 0 }, _Meshed_mask_rows{ 0 };
	HelpMethod::GetValidCalculateMaskSize(grid_step, roi_width, roi_height, &_Meshed_mask_cols, &_Meshed_mask_rows);

	roi_mask = detail::make_cvmat<uint8_t>(_Meshed_mask_rows, _Meshed_mask_cols);
	roi_edge = detail::make_cvmat<uint8_t>(_Meshed_mask_rows, _Meshed_mask_cols * patch_size * patch_size);
	auto _Meshed_mask = cvMat(roi_mask);
	auto _Edge_weight_mat = cvMat(roi_edge);
	helper::_Proc_roi_edge(&_Mask, &_Meshed_mask, &_Edge_weight_mat,
		patch_radius(), grid_step, roi_lt_x, roi_lt_y, validity_ratio);
}

DEFLOW_USE_CV(Mat) get_fundamental_matrix(const options_t& options)
{
	using value_t = options_t::element_type::value_t;
	constexpr auto __type__ = cv::traits::Type<value_t>::value;

	const auto __rt = options->calib_external_pars[0];
	const auto __t = __rt + 3;
	value_t __tx[]={
		0, -__t[2], __t[1], __t[2], 0, -__t[0], -__t[1], __t[0], 0
	};
	auto _Tx = cvMat(3, 3, __type__, __tx);
	auto __r = make_cvmat(3, 1, {__rt[0], __rt[1], __rt[2]});
	__r = __r * (3.14159265358979323846 / 180);
	auto __R = make_cvmat(3, 3, value_t(0));
	DEFLOW_USE_CV(Rodrigues)(__r, __R);
	auto _R = cvMat(__R);
	value_t __e[9];
	auto _E = cvMat(3, 3, __type__, __e);
	cvMatMul(&_Tx, &_R, &_E);

	auto __k = options->calib_internal_pars[1];
	auto __ifx = 1 / __k[0], __icx = -__k[2] / __k[0];
	auto __ify = 1 / __k[1], __icy = -__k[3] / __k[1];

	auto _iKtE = make_cvmat(3, 3, {
		__ifx * __e[0], __ifx * __e[1], __ifx * __e[2],
		__ify * __e[3], __ify * __e[4], __ify * __e[5],
		__icx * __e[0] + __icy * __e[3] + __e[6],
		__icx * __e[1] + __icy * __e[4] + __e[7],
		__icx * __e[2] + __icy * __e[5] + __e[8]
		});

	__k = options->calib_internal_pars[0];
	value_t __ik0[] = {
		1 / __k[0], 0, -__k[2] / __k[0], 0, 1 / __k[1], -__k[3] / __k[1], 0, 0, 1
	};
	auto _iK0 = cvMat(3, 3, __type__, __ik0);
	value_t __f[9];
	auto _F = cvMat(3, 3, __type__, __f);
	_E = cvMat(_iKtE);
	cvMatMul(&_E, &_iK0, &_F);

	const auto __s = abs(__f[8]) > 0 ? 1. / __f[8] : 1;
	return make_cvmat(3, 3, {
		__f[0] * __s, __f[1] * __s, __f[2] * __s,
		__f[3] * __s, __f[4] * __s, __f[5] * __s,
		__f[6] * __s, __f[7] * __s, __f[8] * __s });
}

DEFLOW_ALG bool is_valid_poi(const options_t& opts, const QPointF& poi)
{
	const auto& mask = opts->roi_mask;
	const auto [reduced_x, reduced_y] = opts->reduce_pos(poi.x(), poi.y());
	if (reduced_x<0 || reduced_x>mask.cols || reduced_y<0 || reduced_y>mask.rows) {
		return false;
	}
	if (1 != mask.ptr<uchar>(reduced_y)[reduced_x]) {
		return false;
	}
	return true;
}
}
