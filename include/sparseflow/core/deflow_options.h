/*********************************************************************
 This file is part of Deflow R&D Platform Infrastructure.
 Copyright(C) GOS.VI Lab since 2020, all rights reserved.
*********************************************************************/
#pragma once

#include <QList>
#include <QPoint>

#include <tuple>
#include <type_traits>
#include <opencv2/core/mat.hpp>

#include "sparseflow/__defs_ports__.hpp"
#include "deflow_types.hpp"

namespace deflow {
/// <summary>
/// \brief STRUCT Options, provides computation parameter settings
/// </summary>
struct Options
{
	using value_t = double_t;
	using index_t = int32_t;
	using mask_t = DEFLOW_USE_CV(Mat_<uint8_t>);
	using matrix_t = DEFLOW_USE_CV(Mat_<value_t>);

	// Camera internal parameters, Nx4 matrix '{...,[fx, fy, cx, cy],...}' 
	matrix_t calib_internal_pars;
	// Camera distortion parameters, Nx4 matrix '{...,[k1, k2, p1, p2],...}'
	matrix_t calib_distortn_pars;
	// Camera external parameters, Nx6 matrix '{...,[rx, ry, rz, tx, ty, tz],...}'
	matrix_t calib_external_pars;

	// Reference index for computation
	index_t reference_idx{ 0 };
	// Image path list
	QStringList image_paths;

	// Height of the source image
	index_t imrows{ 1024 };
	// Width of the source image
	index_t imcols{ 1024 };
	Options& set_image_size(index_t rows, index_t cols) noexcept {
		imrows = rows, imcols = cols;
		return (*this);
	}

	// Number of the threads will not managed by the system when enabled
	bool enable_brute_threadify = false;

	// Radius for integer pixel search: D-U-L-R
	index_t search_radius[4]{ 10, 10, 10, 10 };
	bool enable_domain_flwing = false;

	// Epipolar line half-bandwidth
	index_t epline_width{ 5 };
	// Size of sliding window
	index_t patch_size{ 31 };
	// Spacing of the quadrature grid points
	index_t grid_step{ 5 };
	// Max number of iterations
	index_t max_iters{ 20 };
	// Threshold for correlation search
	value_t zncc_level{ 0.8 };
	// Set to record zncc coeffs or not
	bool corr_keep_coefs{ true };
	// 
	bool transfer_init_transform{ true };
	//
	point_search_policy search_policy{ point_search_policy::SHARED_DEF };

	// Build roi_mask and roi_edge
	DEFLOW_ALG void make_roi(const QList<QList<QPointF>>& polys);
	// Left-top corner of rois in image
	index_t roi_lt_x{ 0 }, roi_lt_y{ 0 };
	// Roi width and height in image
	index_t roi_width{ 0 }, roi_height{ 0 };
	// Roi count, equals to size of `polys` given in method `make_roi` 
	index_t roi_count{ 0 };
	// Mask of ROI
	mask_t roi_mask;
	// Edge weight of ROI
	mask_t roi_edge;
	// ROI edge validity
	value_t validity_ratio{ 0.8 };

	// Set to `true` if the displacement is required. Default is `false`.
	bool requires_disp{ false };

	// Strain computation window size
	index_t diff_wnd_size{ 11 };
	// Strain evaluation mode
	strain_calc_tag strain_mode{ strain_calc_tag::BRIEF};
	// Strain evaluation policy
	strain_exec_policy strain_policy{ strain_exec_policy::PAR };

	//< Get image path in sequence given by the index 'idx'
	inline auto path(index_t idx) const noexcept {
		return image_paths[idx].toStdString();
	}

	/**
	 * \brief Get the number of stages.
	 */
	inline auto num_stages() const noexcept {
		return image_paths.size();
	}

	/**
	 * \brief Get radius of the sliding window
	 */
	inline auto patch_radius() const noexcept {
		return patch_size >> 1;
	}

	struct CorrSpec {
		const index_t& stride_x;
		const index_t& stride_y;
		const index_t& max_iters;

		CorrSpec(const index_t &stride_x_, const index_t &stride_y_, const index_t &max_iters_)
			: stride_x(stride_x_), stride_y(stride_y_), max_iters(max_iters_) {}
	};
	inline auto brief_corr()const noexcept {
		return CorrSpec(grid_step, grid_step, max_iters);
	}

	// Map pixel pos from the original image to the meshed ROIs
	template<typename T>
	inline auto reduce_pos(T x, T y) const noexcept {
		return DEFLOW_USE_STD(make_tuple)<index_t,index_t>(
			DEFLOW_USE_STD(round)((x - roi_lt_x) / grid_step),
			DEFLOW_USE_STD(round)((y - roi_lt_y) / grid_step)
			);
	}
	// Map pixel pos from the meshed ROIs to the original image
	inline auto restore_pos(index_t r, index_t c) const noexcept {
		return DEFLOW_USE_STD(make_tuple)(
			roi_lt_x + c*grid_step, roi_lt_y + r*grid_step
			);
	}
};
// Alias type of struct Options
using options_t = DEFLOW_USE_STD(shared_ptr<Options>);

// Factory function to make shared options pointer
inline auto make_shared_options() noexcept {
	return DEFLOW_USE_STD(make_shared<Options>)();
}

// Function to get fundamental matrix
DEFLOW_USE_CV(Mat) get_fundamental_matrix(const options_t& options);

// Check if the given computation point 'poi' is in the given ROIs or not.
DEFLOW_ALG bool is_valid_poi(const options_t& options, const QPointF& poi);
}