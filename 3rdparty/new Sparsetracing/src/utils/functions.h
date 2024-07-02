#pragma once
#include <initializer_list>
#include <random>
#include <opencv2/core.hpp>
#include <qdatetime.h>

#include "sparse/__defs_ports__.hpp"

namespace deflow {
struct Options;

namespace detail {

using image_t = cv::Mat;

template<typename T>
inline auto make_cvmat(size_t rows, size_t cols, T value = T(0)) {
	return image_t(rows, cols, cv::traits::Type<T>::value, cv::Scalar_<T>{ value });
}

template<typename T>
inline auto make_cvmat(int32_t rows, int32_t cols, std::initializer_list<T> values) {
	return image_t({ rows, cols }, values);
}

inline auto have_same_value_type(const image_t& left, const image_t& right) noexcept {
	return left.type() == right.type();
}

/// <summary>
/// \brief Read grayscale image.
/// </summary>
/// <param name="path">Image file path</param>
/// <returns>One-channel uchar cv::Mat</returns>
DEFLOW_ALG image_t imread(const std::string& path);

/// <summary>
/// \brief Generate colormap from a given source image.
/// </summary>
/// <param name="src">Single channel source data image.</param>
/// <param name="opts">Shared pointer to Options instance.</param>
/// <param name="type">Colormap type \sa cv::ColormapType.</param>
/// <returns>A full-size colormap matrix with type of RGBA.</returns>
DEFLOW_ALG image_t make_colormap(image_t src, std::shared_ptr<Options> opts, uint8_t type=2);

/// <summary>
/// \brief Peform bilinear interpolation on regular 2D grid:
///        $f_11=f(x_1, y_1 ), f_12=f(x_1,y_2 ), f_21=f(x_2,y_1 ), f_22=f(x_2,y_2 )$
/// </summary>
/// <param name="node_xy">x and y coordinates of the four nodes [x_1, y_1, x_2, y_2]</param>
/// <param name="value">Function values [f_11, f_12, f_21, f_22] of the four nodes</param>
/// <param name="x">x-coordinate of the pos to be computed.</param>
/// <param name="y">y-coordinate of the pos to be computed.</param>
/// <returns>The interpolated function value f(x,y)</returns>
DEFLOW_ALG double_t bilinear_interp(double_t node_xy[4], double_t value[4], double_t x, double_t y);

/// <summary>
/// \brief Find flaw edge points from a given sample line segment.
/// </summary>
/// <param name="pos">Relative length coordinates of the line.</param>
/// <param name="val">Response values of all sample poinits on the line.</param>
/// <returns>Coord. indices of the sampled flaw edge points</returns>
template<typename T = double_t>
DEFLOW_ALG std::vector<size_t> find_flaw_edge_support(const std::vector<T>& pos, const std::vector<T>& val);

/// <summary>
/// \brief Get number of the available task threads. 
/// </summary>
/// <param name="count">Number of tasks wanna use</param>
/// <returns>Number of tasks can be used.</returns>
DEFLOW_ALG size_t get_num_tasks(size_t count);

} // namespace detail

using detail::imread;
using detail::make_cvmat;
using detail::make_colormap;

} // namespace deflow