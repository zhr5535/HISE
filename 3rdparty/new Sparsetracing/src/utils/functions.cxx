#include "utils/functions.h"
#include "sparse/core/deflow_options.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#if DEFLOW_THR_ENABLE_TBB
#include <tbb/task_group.h>
#include <tbb/task_arena.h>
#endif

namespace deflow {
namespace detail{
DEFLOW_ALG image_t imread(const DEFLOW_USE_STD(string)& path) {
	return DEFLOW_USE_CV(imread)(path, DEFLOW_USE_CV(IMREAD_GRAYSCALE));
}

DEFLOW_ALG size_t get_num_tasks(size_t count)
{
	const auto _Max =
#if DEFLOW_THR_ENABLE_TBB == 1
		tbb::this_task_arena::max_concurrency();
#else
		1;
#endif
	return count < _Max ? count : _Max;
}

DEFLOW_ALG image_t make_colormap(image_t src, std::shared_ptr<Options> opts, uint8_t type)
{
	using index_t = Options::index_t;

	image_t _Src;
	DEFLOW_USE_CV(normalize)(src, _Src, 0, 255, DEFLOW_USE_CV(NORM_MINMAX));
	auto _Ret = make_cvmat(opts->imrows, opts->imcols, uint8_t(0));
	for (auto r = opts->roi_lt_y; r < opts->roi_height; ++r) {
		auto _Ptr = _Ret.ptr<uint8_t>(r);
		const auto reduced_r = float(r - opts->roi_lt_y) / opts->grid_step;
		for (auto c = opts->roi_lt_x; c < opts->roi_width; ++c) {
			const auto reduced_c = float(c - opts->roi_lt_x) / opts->grid_step;
			
			//x = DEFLOW_USE_STD(min)(DEFLOW_USE_STD(max)(0, x), _Src.cols);
			//y = DEFLOW_USE_STD(min)(DEFLOW_USE_STD(max)(0, y), _Src.rows);

		}
	}
	return _Ret;
}

DEFLOW_ALG double_t bilinear_interp(double_t node_xy[4], double_t value[4], double_t x, double_t y)
{
	const auto x1 = node_xy[0], y1 = node_xy[1];
	const auto x2 = node_xy[2], y2 = node_xy[3];
	const auto f11 = value[0], f12 = value[1], f21 = value[2], f22 = value[3];
	const auto factor = 1 / (x2 - x1) / (y2 - y1);
	const auto val_0 = f11 * (y2 - y) + f12 * (y - y1);
	const auto val_1 = f21 * (y2 - y) + f22 * (y - y1);
	const auto val = (x2 - x) * val_0 + (x - x1) * val_1;
	return factor * val;
}

template<typename T>
std::vector<size_t> find_flaw_edge_support(const std::vector<T>& pos, const std::vector<T>& val) {
	std::vector<T> grad;
	for (auto i = 0; i < std::min(pos.size(), val.size()) -1; ++i) {
		const auto der = (val[i + 1] - val[i])/(pos[i+1]-pos[i]);
		grad.push_back(der);
	}
	const auto max_val_it = std::max_element(val.cbegin(), val.cend());
	// If the max response is less than the 5-fold of the normal values,
	// we dont report it as a crack.
	if (*max_val_it < (val.front()+val.back())/2*5) {
		return {};
	}
	// Else we find the edge point with derivative response
	const auto max_val_idx = max_val_it - val.begin();
	const auto first_max_it = std::max_element(grad.cbegin(), grad.cbegin() + max_val_idx);
	const auto secon_max_it = std::max_element(grad.cbegin() + max_val_idx, grad.cend());
	return { size_t(first_max_it - grad.begin()), size_t(secon_max_it - grad.begin()) };
}
}
}

template DEFLOW_ALG std::vector<size_t> deflow::detail::
find_flaw_edge_support<double>(const std::vector<double>&, const std::vector<double>&);
template DEFLOW_ALG std::vector<size_t> deflow::detail::
find_flaw_edge_support<float>(const std::vector<float>&, const std::vector<float>&);