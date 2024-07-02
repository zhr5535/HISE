#include "marker_functions.hpp"
#include "old/imagedetectmethod.h"

namespace dgelom
{
	MarkerDetector::MarkerDetector(size_t rows, size_t cols) noexcept
		: _Myimrows(rows), _Myimcols(cols)
	{
	}

	const MarkerDetector::options_t &MarkerDetector::options() const noexcept
	{
		return (_Myoptions);
	}

	MarkerDetector::options_t &MarkerDetector::options() noexcept
	{
		return (_Myoptions);
	}

	MarkerDetector::marker_desc_cont
	MarkerDetector::detect(const pixel_ptr data) const noexcept
	{
		if (!data)
			return {};
		// Do marker contoure segmentation...
		const auto src = DGE_USE_CV(Mat)(_Myimrows, _Myimcols,
										 dgelom::internal::make_dtype<pixel_t>, data);
		const auto image = detail::_Preprocess(src);
		const auto contours = detail::_Find_contours(_Myoptions.seg_tag, image);

#ifdef _DEBUG
		DGE_USE_STD(remove_const_t)<decltype(image)> _Scene_debug;
		image.convertTo(_Scene_debug, CV_8UC3);
#endif

		// Get ellipse-like contoures...
		int32_t idx = 0;
		marker_desc_cont ellipses;
		for (const auto &_Cont : contours)
		{
			if (_Cont.size() < 6)
				continue;

			const auto elrect = DGE_USE_CV(fitEllipse)(_Cont);
#if __cplusplus <= 202004L
			auto _Desc = marker_desc_type();
			_Desc.id = idx;
			_Desc.cx = elrect.center.x, _Desc.cy = elrect.center.y;
			_Desc.rx = elrect.size.width / 2, _Desc.ry = elrect.size.height / 2;
			_Desc.angle = elrect.angle * 3.141592641f / 180, _Desc.quality = 0;
#else
			auto _Desc = marker_desc_type(decltype(ellipses)::value_type{
				idx, elrect.center.x, elrect.center.y,
				elrect.size.width / 2, elrect.size.height / 2,
				elrect.angle * 3.141592641f / 180, value_t(0)});
#endif
			if (!is_valid(_Desc, _Myoptions))
				continue;

			// Eval elliptic fitting error...
			DGE_USING_STD(accumulate);
			const auto _Err = accumulate(
								  _Cont.begin(), _Cont.end(), value_t(0),
								  [&](auto v, const auto &p)
								  {
									  return v + _Desc.eval(p.x, p.y);
								  }) /
							  _Cont.size();
			// Check with grayscale criterion...
			const auto _Pass = detail::_Is_fits_grayscale_criterion(
				image, _Desc, _Myoptions.k[0]);
			if (_Pass && (_Err < _Myoptions.quality))
			{

				_Desc.quality = 1 - _Err;
				ellipses.push_back(_Desc);
			}
#ifdef _DEBUG
			DGE_USING_CV(drawContours);
			DGE_USING_CV(drawMarker);
			drawContours(_Scene_debug, contours, idx, {255, 0, 0});
			if (!ellipses.empty())
				drawMarker(_Scene_debug, {int(ellipses.back().cx), int(ellipses.back().cy)}, {0, 255, 0});
#endif
			++idx;
		}
		// Remove possible coding sectors...
		for (auto _Pre = ellipses.begin(); _Pre != ellipses.end() - 1; ++_Pre)
		{
			for (auto _Nex = _Pre + 1; _Nex != ellipses.end(); ++_Nex)
			{
				decltype(auto) _Prex = _Pre->cx, _Prey = _Pre->cy;
				decltype(auto) _Nexx = _Nex->cx, _Nexy = _Nex->cy;
				const auto _Dist = sqrt(pow(_Prex - _Nexx, 2) + pow(_Prey - _Nexy, 2));
				DGE_USING_STD(min);
				if (_Dist < min(_Pre->ry, _Nex->ry))
				{
					if (_Pre->ry > _Nex->ry)
					{
						_Pre = ellipses.erase(_Pre);
						//--_Pre;
						break;
					}
					else
					{
						_Nex = ellipses.erase(_Nex);
						//--_Nex;
					}
				}
				if (_Dist < min(_Pre->ry, _Pre->ry * _Myoptions.k[2]))
				{
					if (_Pre->quality < _Nex->quality)
					{
						_Pre = ellipses.erase(_Pre);
						//--_Pre;
						break;
					}
					else
					{
						_Nex = ellipses.erase(_Nex);
						//--_Nex;
					}
				}
			}
		}

		// Decoding. Regard the uncoded markers as special coded ones with ID -1.
		const auto _Ids = _Get_decode_ids_b15();
		for (auto _It = ellipses.begin(); _It != ellipses.end(); ++_It)
		{
			const auto _Ellip_a2 = _It->rx * _Myoptions.k[1];
			const auto _Ellip_b2 = _It->ry * _Myoptions.k[1];
			const auto _Ellip_a4 = _It->rx * _Myoptions.k[2];
			const auto _Ellip_b4 = _It->ry * _Myoptions.k[2];

			const auto _Top = int(_It->cy - _Ellip_b4);
			const auto _Bot = int(_It->cy + _Ellip_b4);
			const auto _Lef = int(_It->cx - _Ellip_b4);
			const auto _Rig = int(_It->cx + _Ellip_b4);
			auto _Id = -1;
			if (_Top < 0 || _Bot > image.rows - 11 || _Lef < 0 || _Rig > image.cols - 1)
			{
				_Id = -2;
			}
			const auto code_bits = 15;
			const auto dot_per_sector = 20;
			DGE_USING_STD(sin);
			DGE_USING_STD(cos);
			const auto sine = sin(_It->angle), cosine = cos(_It->angle);
			DGE_USE_STD(vector)<DGE_USE_CV(Point2f)> _Points;
			DGE_USE_STD(vector)<int32_t> _Grayvals;
			for (auto _Idx = 0; _Idx < code_bits * dot_per_sector; ++_Idx)
			{
				const auto _Ang = (_Idx << 1) * 3.1415926f / (code_bits * dot_per_sector);

				const auto x = cos(_Ang), y = sin(_Ang);
				_Points.push_back({x, y});

				const auto _Midx = _Ellip_a2 * x, _Midy = _Ellip_b2 * y;
				const auto _Outx = _Ellip_a4 * x, _Outy = _Ellip_b4 * y;
				const auto _Midx_ = _Midx * cosine - _Midy * sine + _It->cx;
				const auto _Midy_ = _Midx * sine + _Midy * cosine + _It->cy;
				const auto _Outx_ = _Outx * cosine - _Outy * sine + _It->cx;
				const auto _Outy_ = _Outx * sine + _Outy * cosine + _It->cy;

				const auto _P1 = decltype(_Points)::value_type{_Midx_, _Midy_};
				const auto _P2 = decltype(_Points)::value_type{_Outx_, _Outy_};

				const auto _Diff = _P2 - _P1;
				const auto _K = _Diff.y / _Diff.x;
				const auto _Inc = _Diff.x / abs(_Diff.x);
				decltype(_Grayvals) _Temp;
				_Temp.reserve(100);
				for (auto _Cx = _P1.x; _Cx <= _P2.x; _Cx += _Inc)
				{
					const auto _Cy = _K * (_Cx - _P1.x) + _P1.y;
					_Temp.push_back(image.ptr(_Cy)[size_t(_Cx)]);
				}
				DGE_USING_STD(sort);
				sort(DGE_USE_STD(execution)::par, _Temp.begin(), _Temp.end());
				if (const auto _Mid = _Temp.size() >> 1; _Mid % 2 == 0)
				{
					_Grayvals.push_back((_Temp[_Mid] + _Temp[_Mid - 1]) >> 1);
				}
				else
				{
					_Grayvals.push_back(_Temp[_Mid]);
				}
			}

			// Find boundary of the unit circle codes.
		}

		return DGE_USE_STD(forward)<decltype(ellipses)>(ellipses);
	}

	MarkerDetector::marker_desc_cont MarkerDetector::exect(const pixel_ptr data) const
	{
		cv::Mat _Coded, _Uncod;
		ImageDetectMethod::_General_circular_marker_detector(
			{data, _Myimrows, _Myimcols},
			_Coded, _Uncod,
			options().min_radius, options().max_radius,
			(float *)options().k,
			options().quality,
			options().forcolor == 0 ? BlackDownWhiteUp : WhiteDownBlackUp,
			CodePointBitesType::CodeBites15,
			DetectContoursMethod(options().seg_tag),
			subpix_refine_tag::Gray_Centroid,
			options().enhanced);
		marker_desc_cont _Ret(_Coded.rows + _Uncod.rows);
		for (auto _Idx = 0; _Idx < _Uncod.rows; ++_Idx)
		{
			const auto _Par = _Uncod.ptr<float>(_Idx);
			_Ret[_Idx].cx = _Par[1];
			_Ret[_Idx].cy = _Par[2];
			_Ret[_Idx].quality = _Par[3];
			_Ret[_Idx].rx = _Par[4];
			_Ret[_Idx].ry = _Par[5];
			_Ret[_Idx].angle = _Par[6];
		}
		for (auto _Idx = 0; _Idx < _Coded.rows; ++_Idx)
		{
			const auto _Par = _Coded.ptr<float>(_Idx);
			_Ret[_Idx + _Uncod.rows].id = _Par[0];
			_Ret[_Idx + _Uncod.rows].cx = _Par[1];
			_Ret[_Idx + _Uncod.rows].cy = _Par[2];
			_Ret[_Idx + _Uncod.rows].quality = _Par[3];
			_Ret[_Idx + _Uncod.rows].rx = _Par[4];
			_Ret[_Idx + _Uncod.rows].ry = _Par[5];
			_Ret[_Idx + _Uncod.rows].angle = _Par[6];
		}

		return _Ret;
	}

	MarkerDetector::value_t
	MarkerDetector::marker_desc_type::eval(value_t x, value_t y) const noexcept
	{
		const auto [tx, ty] = detail::_Cvto_elliptic_coords(x, y, *this);

		const auto alpha = atan2(ty, tx);
		const auto r = rx * ry / sqrt(rx * rx * pow(sin(alpha), 2) + ry * ry * pow(cos(alpha), 2));
		const auto diff = sqrt(tx * tx + ty * ty) - r;

		return abs(diff);
	}

	bool is_valid(MarkerDetector::marker_desc_type desc, MarkerDetector::options_t options) noexcept
	{
		const auto min_radius_cond = options.min_radius < desc.rx;
		const auto max_radius_cond = options.max_radius > desc.ry;
		const auto axis_ratio_cond = desc.ry / desc.rx <= 2;

		return min_radius_cond && max_radius_cond && axis_ratio_cond;
	}

}
