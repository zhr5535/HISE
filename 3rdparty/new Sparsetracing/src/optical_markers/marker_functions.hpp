#pragma once
#include <numeric>
#include <algorithm>
#include <execution>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "marker_detector.h"

namespace dgelom
{
	namespace internal
	{
		template <typename T>
		inline static constexpr auto make_dtype = DGE_USE_CV(DataType)<T>::generic_type;
	}
	namespace detail
	{

		using DGE_USE_STD(vector);
		using DGE_USE_CV(Point);
		using DGE_USE_CV(Mat);
		using mdtor_t = MarkerDetector;

		inline Mat
		_Preprocess(const Mat &image)
		{
			DGE_USE_STD(decay_t)<decltype(image)> _Ret;
			DGE_USING_CV(GaussianBlur);
			GaussianBlur(image, _Ret, {3, 3}, 1.5, 1.5);
			return DGE_USE_STD(move)(_Ret);
		}

		inline vector<vector<Point>>
		_Find_contours(mdtor_t::threshold_type tag, const Mat &image)
		{
			Mat binary_image;
			vector<DGE_USE_CV(Vec4i)> hierarchy;
			switch (tag)
			{
			case mdtor_t::threshold_type::TRIA:
				DGE_USING_CV(threshold);
				threshold(image, binary_image, 0, 255,
						  DGE_USE_CV(THRESH_TRIANGLE) | DGE_USE_CV(THRESH_BINARY_INV));
				break;
			case mdtor_t::threshold_type::OTSU:
				DGE_USING_CV(threshold);
				threshold(image, binary_image, 0, 255,
						  DGE_USE_CV(THRESH_OTSU) | DGE_USE_CV(THRESH_BINARY_INV));
				break;
			case mdtor_t::threshold_type::MEAN:
				DGE_USING_CV(adaptiveThreshold);
				adaptiveThreshold(image, binary_image, 255,
								  DGE_USE_CV(ADAPTIVE_THRESH_MEAN_C), DGE_USE_CV(THRESH_BINARY), 5, 0);
				break;
			default:
				DGE_USING_CV(adaptiveThreshold);
				adaptiveThreshold(image, binary_image, 0, 255,
								  DGE_USE_CV(ADAPTIVE_THRESH_GAUSSIAN_C) | DGE_USE_CV(THRESH_BINARY_INV), 5, 0);
				break;
			}
			vector<vector<Point>> contours;
			DGE_USING_CV(findContours);
			findContours(binary_image, contours, DGE_USE_CV(RETR_LIST),
						 DGE_USE_CV(CHAIN_APPROX_NONE), {0, 0});

			return DGE_USE_STD(forward)<decltype(contours)>(contours);
		}

		inline auto
		_Cvto_elliptic_coords(mdtor_t::value_t x, mdtor_t::value_t y, const mdtor_t::marker_desc_type &desc) noexcept
		{
			using DGE_USE_STD(cos);
			using DGE_USE_STD(sin);

			const auto &angle = desc.angle;
			const auto c = cos(angle), s = sin(angle);
			const auto tx = (x - desc.cx) * c + (y - desc.cy) * s;
			const auto ty = -(x - desc.cx) * s + (y - desc.cy) * c;

			return DGE_USE_STD(tuple)(tx, ty);
		}

		inline bool
		_Is_fits_grayscale_criterion(const Mat &image, const mdtor_t::marker_desc_type &desc, mdtor_t::value_t ratio) noexcept
		{
			using DGE_USE_STD(max);
			using DGE_USE_STD(min);
			const auto ela = desc.rx * ratio, elb = desc.ry * ratio;
			const auto top = max(desc.cy - elb, 0.f);
			const auto bottom = desc.cy + elb > image.rows - 1 ? 0 : desc.cy + elb;
			const auto left = max(desc.cx - elb, 0.f);
			const auto right = desc.cx + elb > image.cols - 1 ? 0 : desc.cy + elb;

			vector<decltype(ratio)> bgvals, fgvals;
			for (auto y = top; y <= bottom; ++y)
			{
				const auto _Ptr = image.ptr(y);
				for (auto x = left; x <= right; ++x)
				{
					const auto [ex, ey] = detail::_Cvto_elliptic_coords(x, y, desc);
					const auto _Val1 = ex * ex / (ela * ela) + ey * ey / (elb * elb);
					const auto _Val2 = ex * ex / (desc.rx * desc.rx) + ey * ey / (desc.ry * desc.ry);
					if (_Val1 < 1 && _Val2 > 1)
					{
						bgvals.push_back(_Ptr[size_t(x)]);
					}
					if (_Val2 < 1)
					{
						fgvals.push_back(_Ptr[size_t(x)]);
					}
				}
			}
			constexpr auto policy = DGE_USE_STD(execution)::par;
			DGE_USING_STD(reduce);
			const auto fg_sum = reduce(policy, fgvals.begin(), fgvals.end(), 0.f);
			const auto bg_sum = reduce(policy, bgvals.begin(), bgvals.end(), 0.f);
			// const auto tol_mean = (fg_sum+bg_sum)/(fgvals.size()+bgvals.size());
			const auto fg_mean = fg_sum / fgvals.size();
			const auto bg_mean = bg_sum / bgvals.size();
			DGE_USING_STD(forward);
			auto fg_std = decltype(fg_mean){0};
			for_each(policy, fgvals.begin(), fgvals.end(),
					 [&fg_std, fg_mean](const auto &v)
					 {
						 fg_std += (v - fg_mean) * (v - fg_mean);
					 });
			fg_std = sqrt(fg_std / fgvals.size());
			auto bg_std = decltype(bg_mean){0};
			for_each(policy, bgvals.begin(), bgvals.end(),
					 [&bg_std, bg_mean](const auto &v)
					 {
						 bg_std += (v - bg_mean) * (v - bg_mean);
					 });
			bg_std = sqrt(bg_std / bgvals.size());
			/*auto tol_std = decltype(tol_mean){0};
			for_each(policy, fgvals.begin(), fgvals.end(),
				[&tol_std, tol_mean](const auto& v) {
					tol_std += (v - tol_mean) * (v - tol_mean);
				});
			tol_std = reduce(policy, bgvals.begin(), bgvals.end(), tol_std,
				[tol_mean](auto _V, const auto& _U) {
					return _V + (_U - tol_mean) * (_U - tol_mean);
				});
			tol_std = sqrt(tol_std / bgvals.size()+fgvals.size());*/

			if (abs(fg_mean - bg_mean) < 50)
				return false;
			if (fg_std > 100)
				return false;
			if (bg_std > 100)
				return false;
			return true;
		}

		inline vector<int32_t>
		_Batch_decoding(const mdtor_t::marker_desc_cont &descs, mdtor_t::options_t options)
		{
		}
	} // namespace detail
} // namespace dgelom