#include <QPainter>
#include <QPrinter>
#include <QPainterPath>
#include <QFileDialog>
#include <QMessageBox>

#include <omp.h>
#include <mutex>
#include <thread>
#include <cmath>
#include <math.h>

#include "utils.h"
#include "optical_markers.h"
#include "marker_detector.h"
#include "internal/authorization.hpp"

#define MAKE_PREFIX(TAG) \
	OpticalMarkers<om_marker_tag::##TAG>

namespace dgelom
{
	namespace detail
	{
		template <typename T>
		auto _Get_marker_radii(T _Scale) noexcept
		{
			return std::tuple(2.5 * _Scale, 6 * _Scale, 10 * _Scale);
		}
		template <typename T>
		auto _Get_single_rect_size(T _Scale) noexcept
		{
			return 30 * _Scale;
		}
		template <typename T>
		auto _Cvt_decimal_to_binary(T _Val, size_t _Bits) noexcept
		{
			QList<size_t> _Ret;
			auto _Rest = _Val;
			for (auto i = 0; i < _Bits; i++)
			{
				_Ret.prepend(_Rest % 2);
				_Rest = _Rest >> 1;
			}
			return _Ret;
		}
		auto _Cvt_binary_to_decimal(const QList<size_t> &binary_codes) noexcept
		{
			size_t _Ans = 0;
			const auto n = binary_codes.size();
			for (auto i = 0; i < n; i++)
			{
				_Ans = _Ans + size_t(pow(2.0, n - i - 1) * binary_codes.value(i));
			}
			return _Ans;
		}

		struct Sector
		{
			using value_t = float;
			value_t start_angle{0};
			value_t sweep_angle{0};

			struct FixedSector
			{
				const value_t &start_angle;
				const value_t &sweep_angle;
				value_t cx, cy;

				FixedSector(const value_t &stagl, const value_t &swagl, value_t x, value_t y)
					: start_angle(stagl), sweep_angle(swagl), cx(x), cy(y)
				{
				}

				template <typename _Rect>
				inline auto rect(value_t radius) const noexcept
				{
					return _Rect(cx - radius, cy - radius, 2 * radius, 2 * radius);
				}
			};

			inline auto moveto(value_t cx, value_t cy) const noexcept
			{
				return FixedSector(start_angle, sweep_angle, cx, cy);
			}
		};
		template <typename _Int>
#if __cplusplus > 202004L
			requires std::integral<_Int>
#endif
		auto _I2B(_Int id, size_t nbits) noexcept
		{
			std::vector<bool> _Codes(nbits);
			for (decltype(auto) _Val : _Codes)
			{
				_Val = id % 2;
				id = id >> 1;
			}
			return _Codes;
		}
		/// <summary>
		/// \brief Generate coding sectors for n-bits circular coding markers (CCMs).
		/// </summary>
		/// <param name="nbits">bits of the CCMs.</param>
		/// <param name="max">max ID of the CCMs to be generated.</param>
		/// <param name="min">min ID of the CCMs to be generated, default is 0.</param>
		/// <returns>Sectors list with type std::vector of std::vector of Sector.</returns>
		auto _Coding_sectors(size_t nbits, size_t max, size_t min = 0) noexcept
		{
			using sectors_t = std::vector<Sector>;
			const auto _Min = std::max(min, size_t(0));
			const auto _Max = std::min(max, size_t(std::pow(2, nbits)));
			const auto _Unit = Sector::value_t(360) / nbits;
			std::vector<sectors_t> _Ret;
			for (auto _Id = _Min; _Id < _Max; ++_Id)
			{
				sectors_t _Sectors;
				decltype(auto) _Codes = _I2B(_Id, nbits);
				for (auto _It = _Codes.begin(); _It != _Codes.end();)
				{
					auto _Sector = decltype(_Sectors)::value_type();
					_It = std::find(_It, _Codes.end(), true); // Find the starting iterator of the coding pie
					if (_It != _Codes.end())
					{ // Find a coding pie, collect it...
						_Sector.start_angle = (_It - _Codes.begin()) * _Unit;
						for (; *_It; ++_It)
						{
							_Sector.sweep_angle += _Unit;
						}
					}
					_Sectors.push_back(_Sector);
				}
				_Ret.push_back(_Sectors);
			}
			return _Ret;
		}

		template <size_t _Size>
		struct _Page_geometry
		{
		};
		template <>
		struct _Page_geometry<size_t(QPageSize::A4)>
		{
			float resolution;
			size_t mark_size;
			size_t space;

			explicit _Page_geometry(float ppm, size_t rect_size, size_t gap) noexcept
			{
				resolution = ppm;
				mark_size = rect_size * ppm;
				space = gap * ppm;
			}

			inline size_t w() const noexcept
			{
				return 210 * resolution;
			}
			inline size_t h() const noexcept
			{
				return 297 * resolution;
			}
			// \brief Page size of A4 pager.
			inline auto size() const noexcept
			{
				return w() * h();
			}
			// \brief Grid rect size of each marker.
			inline auto elsize() const noexcept
			{
				return mark_size + space;
			}
			inline size_t margin_x() const noexcept
			{
				return (w() - (w() / elsize()) * elsize() - space) >> 1;
			}
			inline size_t margin_y() const noexcept
			{
				return (h() - (h() / elsize()) * elsize() - space) >> 1;
			}
		};
	}
	/// <summary>
	/// \brief Marker encoding impl.
	/// </summary>
	/// <param name="options"></param>
	/// <returns></returns>
	const marker_data_type &
	OpticalMarkers<om_marker_tag::CODED>::encode(options_type options) noexcept
	{
		_Myoptions = options;
		// Set control sizes of the makers
		constexpr auto ppmm = _Mydpi<> / (_Mydpi<float> / 10); // pixels per millimeter
		const auto scale = _Myoptions.inner_radius / 2.5;
		const auto [r1, r2, r3] = detail::_Get_marker_radii(scale);
		const auto mark_rect_size = detail::_Get_single_rect_size(scale);
		const auto space = _Myoptions.spacing;
		const auto r1_pix = size_t(r1 * ppmm);
		const auto r2_pix = size_t(r2 * ppmm);
		const auto r3_pix = size_t(r3 * ppmm);
		const auto page = detail::_Page_geometry<QPageSize::A4>{
			ppmm, size_t(mark_rect_size /* * ppmm*/), size_t(space /* * ppmm*/)};
		_Mysize = mark_rect_size;
		_Mydata.page_size.setWidth(page.w());
		_Mydata.page_size.setHeight(page.h());

		// Generate code ids
		QList<size_t> decimal_code_ids;
		QList<decltype(decimal_code_ids)> binary_code_ids;
		const auto dummy_id_array = _Get_dummy_code_ids_b15();
		const auto start_id = _Myoptions.start_id;
		const auto end_id = _Myoptions.end_id;
		for (auto id = start_id; id < end_id + 1; ++id)
		{
			if (id < dummy_id_array.size())
			{
				decimal_code_ids << id;
				binary_code_ids << detail::_Cvt_decimal_to_binary(
					dummy_id_array[id], _Myoptions.code_bits);
			}
		}

		// Generate mark point context
		const auto num_per_row = page.w() / page.elsize(); // number of markers per row
		const auto num_per_col = page.h() / page.elsize(); // number of markers per column
		const auto num_per_page = num_per_row * num_per_col;
		if (num_per_page == 0)
		{
			return _Mydata;
		}
		const size_t num_pages = ceil(decimal_code_ids.size() / float(num_per_page));

		for (auto page_idx = 0; page_idx < num_pages; ++page_idx)
		{
			QVector<om_marker_type> markers;
			for (auto row = 0; row < num_per_col; ++row)
			{
				const auto global_row = page_idx * num_per_page + row * num_per_row;
				for (auto col = 0; col < num_per_row; ++col)
				{
					const auto marker_idx = global_row + col;
					if (marker_idx >= decimal_code_ids.size())
					{
						break;
					}

					om_marker_type marker;

					// Generate the rect area for each marker
					marker.outer = QRectF(
						page.elsize() * col + page.space + page.margin_x(),
						page.elsize() * row + page.space + page.margin_y(),
						page.mark_size, page.mark_size);

					// Center of the marker being generated
					const auto cx = page.elsize() * col + page.mark_size / 2.f + page.margin_x() + page.space;
					const auto cy = page.elsize() * row + page.mark_size / 2.f + page.margin_y() + page.space;

					marker.center = {cx - r1_pix, cy - r1_pix, r1_pix * 2., r1_pix * 2.};
					marker.middle = {cx - r2_pix, cy - r2_pix, r2_pix * 2., r2_pix * 2.};
					marker.label.pos = {
						col * page.elsize() + 10 * scale + page.space + page.margin_x(),
						row * page.elsize() + 30 * scale + page.space + page.margin_y()};
					marker.label.font = QFont("times", 25 * scale, QFont::Bold);
					marker.label.text = QString::number(decimal_code_ids[marker_idx]);

					// Generate coding sectors...
					bool has_sweep_angle = false;
					int32_t start_angle = 0, sweep_angle = 0;
					const auto &binary_codes = binary_code_ids[marker_idx];
					const auto step_angle = 16 * 360 / binary_codes.size();
					for (auto n = binary_codes.size() - 1; n >= 0; --n)
					{
						if (binary_codes[n] == 1)
						{
							if (!has_sweep_angle)
							{
								has_sweep_angle = true;
								start_angle = (binary_codes.size() - 1 - n) * step_angle;
							}
							sweep_angle += step_angle;
						}
						else
						{
							if (has_sweep_angle)
							{
								has_sweep_angle = false;
								marker.sectors.push_back({cx, cy, r3_pix, start_angle, sweep_angle});
								sweep_angle = 0;
							}
						}
						if (n == 0)
						{
							if (has_sweep_angle)
							{
								has_sweep_angle = false;
								marker.sectors.push_back({cx, cy, r3_pix, start_angle, sweep_angle});
								sweep_angle = 0;
							}
						}
					}
					markers.push_back(marker);
				}
			}
			_Mydata.data.push_back(markers);
		}
		return (_Mydata);
	}

	/// <summary>
	/// \brief Convert markers to images.
	/// </summary>
	/// <returns></returns>
	QPixmap OpticalMarkers<om_marker_tag::CODED>::to_image() noexcept
	{
		return forward_to_image(_Mydata);
	}

	/// <summary>
	/// \brief Convert markers to PDF file.
	/// </summary>
	/// <returns></returns>
	void OpticalMarkers<om_marker_tag::CODED>::to_pdf() noexcept
	{
		return forward_to_pdf(_Mydata, _Myoptions.inner_radius);
	}

	/// <summary>
	/// \brief Decoding the markers
	/// </summary>
	/// <param name="image"></param>
	/// <returns></returns>
	OpticalMarkers<om_marker_tag::CODED>::marker_desc_cont
	OpticalMarkers<om_marker_tag::CODED>::
		decode(const image_pointer image) noexcept
	{
		const auto _Lic_pool = QStringList{
			"11KV-R90B-06C3-S5SV", // This host
			"067/-80F1-0652-NN04", // Lixin Li
			"1T7G-PF1E-06EA-0025", // Zhenyang Yu
			"1A3/-1M8D-06C1-E823", // XPS 13
			"8KK5-YX05-06A3-E823", // Shuo Wang
		};
		const auto _Id = dgelom::make_identifier_code();
		const auto _Ret = std::find(_Lic_pool.begin(), _Lic_pool.end(), _Id);
		const auto _Authorized = _Ret != _Lic_pool.end();
		if (_Authorized || !is_expired({2024, 12, 31}))
		{
			const auto &[imptr, rows, cols] = image;
			auto detector = MarkerDetector(rows, cols);
			detector.options().seg_tag = MarkerDetector::threshold_type(_Myoptions.segmentation);
			detector.options().min_radius = _Myoptions.min_radius;
			detector.options().max_radius = _Myoptions.max_radius;
			detector.options().enhanced = _Myoptions.enable_hf;
			const auto _Cont = detector.exect(imptr);
			_Mycoded.clear();
			_Myuncoded.clear();
			for (const auto &m : _Cont)
			{
				m.id < 0 ? _Myuncoded.push_back(m) : _Mycoded.push_back(m);
			}
			_Myuncoded.shrink_to_fit();
			_Mycoded.shrink_to_fit();
			return _Mycoded;
		}
		else
		{
			QString text = "\
Compatibility issues with the target host architecture have been detected. \
Please contact the vendor with the following error code: %1.";
			_Mymsg = text.arg(dgelom::make_identifier_code());
			return {};
		}
	}

	const OpticalMarkers<om_marker_tag::CODED>::marker_desc_cont &
	OpticalMarkers<om_marker_tag::CODED>::get_coded_markers() const noexcept
	{
		return _Mycoded;
	}

	const OpticalMarkers<om_marker_tag::CODED>::marker_desc_cont &
	OpticalMarkers<om_marker_tag::CODED>::get_uncoded_markers() const noexcept
	{
		return _Myuncoded;
	}

	OpticalMarkers<om_marker_tag::CODED>::marker_desc_cont
	OpticalMarkers<om_marker_tag::CODED>::get_reserved(marker_desc_cont &markers) noexcept
	{
		marker_desc_cont _Reserved;
		for (auto _Idx = 0; _Idx < 4; ++_Idx)
		{
			auto _It = std::find_if(markers.begin(), markers.end(),
									[&_Idx](const auto &_P)
									{
										return _P.id == _Idx;
									});
			if (_It != markers.end())
			{
				_Reserved.push_back(*_It);
				markers.erase(_It);
			}
		}
		return _Reserved;
	}

	/// <summary>
	/// \brief Get marker size.
	/// </summary>
	/// <returns></returns>
	size_t OpticalMarkers<om_marker_tag::CODED>::marker_size() const noexcept
	{
		return _Mysize;
	}

	QPixmap forward_to_image(const marker_data_type &data, size_t idx) noexcept
	{
		if (data.data.isEmpty())
			return QPixmap();
		const auto &marker_page = data.data[idx];
		const auto forcolor = Qt::GlobalColor(data.forcolor);
		const auto bgcolor = (forcolor == Qt::white) ? Qt::black : Qt::white;

		auto painter = QPainter();
		auto image = QImage(data.page_size.width(), data.page_size.height(),
							QImage::Format_ARGB32);
		image.fill(Qt::white);
		painter.begin(&image);
		painter.setPen(Qt::NoPen);
		painter.setRenderHint(QPainter::Antialiasing);
		for (int32_t i = 0; i < marker_page.size(); ++i)
		{
			decltype(auto) marker = marker_page[i];
			const auto uncoded = marker.sectors.empty();
			QPainterPath rect;
			rect.addRect(marker.outer);
			if (uncoded)
			{
				rect.addText(marker.label.pos, marker.label.font, marker.label.text);
			}
			// Fill outer background rects
			painter.fillPath(rect, uncoded ? forcolor : bgcolor);
			// Draw coding rings
			painter.setBrush(forcolor);
			for (const auto &code : marker.sectors)
			{
				painter.drawPie(code.rect<QRectF>(), code.start, code.span);
			}
			// Fill middle background rings
			QPainterPath middle_ring;
			middle_ring.addEllipse(marker.middle);
			painter.fillPath(middle_ring, bgcolor);
			// Fill inner circles and marker label
			QPainterPath center;
			center.addEllipse(marker.center);
			if (!uncoded)
				center.addText(marker.label.pos, marker.label.font, marker.label.text);
			painter.fillPath(center, forcolor);
		}
		return QPixmap::fromImage(std::move(image));
	}

	void forward_to_pdf(const marker_data_type &markers, float cr) noexcept
	{
		if (markers.data.empty())
		{
			return;
		}
		const auto uncoded = markers.data.front().front().sectors.empty();
		const auto fname = QString("%1-markers r-%2 mm.pdf").arg(uncoded ? "uncoded" : "coded").arg(cr);
		const auto path = QFileDialog::getSaveFileName(nullptr,
													   "Export to PDF...", fname, "PDF file(*.pdf)");
		if (path.isEmpty())
			return;

		auto printer = QPrinter();
		printer.setResolution(QPrinter::HighResolution);
		printer.setResolution(254);
		printer.setOutputFormat(QPrinter::PdfFormat);
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
		printer.setPageMargins(0, 0, 0, 0, QPrinter::DevicePixel);
		printer.setPageSize(QPagedPaintDevice::A4);
#else
		printer.setPageMargins({0, 0, 0, 0}, QPageLayout::Unit::Point);
		printer.setPageSize(/*QPageSize*/ QPrinter::A4);
#endif
		printer.setOutputFileName(path);

		auto painter = QPainter(&printer);
		const auto forcolor = Qt::GlobalColor(markers.forcolor);
		const auto bgcolor = (forcolor == Qt::white) ? Qt::black : Qt::white;
		for (auto idx = 0; idx < markers.data.size(); ++idx)
		{
			for (const auto &marker : markers.data[idx])
			{
				QPainterPath rect;
				rect.addRect(marker.outer);
				if (uncoded)
				{
					rect.addText(marker.label.pos, marker.label.font, marker.label.text);
				}
				// Fill outer background rects
				painter.fillPath(rect, uncoded ? forcolor : bgcolor);
				// Draw coding rings
				painter.setBrush(forcolor);
				for (const auto &code : marker.sectors)
				{
					painter.drawPie(code.rect<QRectF>(), code.start, code.span);
				}
				// Fill middle background rings
				QPainterPath middle_ring;
				middle_ring.addEllipse(marker.middle);
				painter.fillPath(middle_ring, bgcolor);
				// Fill inner circles and marker label
				QPainterPath center;
				center.addEllipse(marker.center);
				center.addText(marker.label.pos, marker.label.font, marker.label.text);
				painter.fillPath(center, forcolor);
			}
			if (idx < markers.data.size() - 1)
				printer.newPage();
		}
		painter.end();
	}

	const marker_data_type &UncodedMarkers::generate() noexcept
	{
		const auto ppmm = options().dpi / 25.4f;
		const auto r1 = options().radius;
		const auto scale = r1 / 2.5;
		const auto r2 = scale * 6;
		const auto rect_size = 16 * scale;
		const auto space = options().spacing;
#if QT_VERSION < QT_VERSION_CHECK(5, 15, 0)
		const auto page = detail::_Page_geometry<size_t(QPagedPaintDevice::A4)>(
			ppmm, size_t(rect_size), size_t(space));
#else
		const auto page = detail::_Page_geometry<QPageSize::A4>(
			ppmm, size_t(rect_size), size_t(space));
#endif
		const auto r1_pix = size_t(r1 * ppmm);
		const auto r2_pix = size_t(r2 * ppmm);
		const auto num_per_row = page.w() / page.elsize();
		const auto num_per_col = page.h() / page.elsize();
		const auto num_per_page = num_per_row * num_per_col;

		// Generate uncoded markers on a single page
		QVector<om_marker_type> markers(num_per_page);
		for (auto row = 0; row < num_per_col; ++row)
		{
			const auto row_idx = row * num_per_row;
#pragma omp parallel for
			for (auto col = 0; col < num_per_row; ++col)
			{
				const auto idx = row_idx + col;
				om_marker_type marker;
				marker.outer = QRectF(
					page.elsize() * col + page.space + page.margin_x(),
					page.elsize() * row + page.space + page.margin_y(),
					qreal(page.mark_size), qreal(page.mark_size));
				const auto shift = page.mark_size / 2.f + page.space;
				const qreal cx = page.elsize() * col + page.margin_x() + shift;
				const qreal cy = page.elsize() * row + page.margin_y() + shift;
				marker.center = {
					cx - r1_pix, cy - r1_pix, r1_pix * 2.f, r1_pix * 2.f};
				marker.middle = {
					cx - r2_pix, cy - r2_pix, r2_pix * 2.f, r2_pix * 2.f};
				marker.label.pos = {
					col * page.elsize() + 5 * scale + page.space + page.margin_x(),
					row * page.elsize() + 10 * scale + page.space + page.margin_y()};
				marker.label.font = QFont("times", 5 * scale, QFont::Bold);
				marker.label.text = "DGELOM";
				markers[idx] = marker;
			}
		}
		_Mymarkers.data.push_back(markers);
		_Mymksize = rect_size;
		_Mymarkers.page_size = {int(page.w()), int(page.h())};
		_Mymarkers.forcolor = _Mybase::options().color;
		return (_Mymarkers);
	}

	QPixmap UncodedMarkers::to_image() const noexcept
	{
		return forward_to_image(_Mymarkers);
	}

	void UncodedMarkers::to_pdf() const noexcept
	{
		return forward_to_pdf(_Mymarkers, _Mybase::options().radius);
	}

} // namespace dgelom
