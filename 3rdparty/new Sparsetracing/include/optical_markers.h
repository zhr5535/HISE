/******************************************************************
  This file is part of DGELOM(C) 3D-DeFlow 2021 Application.
  Copyright(C) 2019-2021, Zhilong Su, all rights reserved.
******************************************************************/
#pragma once

#include "marker_fwd.h"
#include <vector>
#include <memory>
#include <QRect>
#include <QFileDialog>
#include <QTextStream>
#include <QString>
#include <QProcess>
#include <QDate>
#include <QObject>

class QImage;
class QPrinter;
namespace dgelom
{
	struct om_marker_type
	{
		struct label_t
		{
			QPointF pos;
			QFont font;
			QString text;
		};
		QRectF center; // rect of the central circle
		QRectF middle;
		QRectF outer; // grid rect of markers
		label_t label;
		QVector<om_code_sector> sectors; // used for coded markers
	};

	struct marker_data_type
	{
		size_t forcolor = Qt::white;
		QSize page_size;
		QList<QVector<om_marker_type>> data;
	};

	class Markers
	{
		using _Myt = Markers;

	public:
		using value_t = float;
		using index_t = size_t;
		using image_pointer = std::tuple<u_char *, index_t, index_t>;

		struct Options
		{
			value_t radius{2.5};
			value_t spacing{0.5};
			index_t start_id{0}, end_id{53}; // for coded markers
			index_t code_bits{15};			 // 15-bit for default
			index_t color{Qt::white};		 // marker color (foreground)
			om_marker_tag type{om_marker_tag::CODED};

			index_t dpi{254};
		};
		Markers() = default;
		Markers(Options &&opts) noexcept : _Myoptions{opts}
		{
		}

		inline _Myt &marker_type(om_marker_tag tag) noexcept
		{
			_Myoptions.type = tag;
			return (*this);
		}
		inline decltype(auto) options() const noexcept
		{
			return (_Myoptions);
		}
		inline decltype(auto) options() noexcept
		{
			return (_Myoptions);
		}
		inline decltype(auto) msize() const noexcept
		{
			return _Mymksize;
		}
		decltype(auto) generate(Options &&opts) noexcept
		{
			_Myoptions = opts;
			return this->generate();
		}

		/*Following virtual methods must be overrided in the derived classes*/
		virtual const marker_data_type &generate() noexcept = 0;
		virtual QPixmap to_image() const noexcept = 0;
		virtual void to_pdf() const noexcept = 0;

	protected:
		size_t _Mymksize{0};
		Options _Myoptions;
		marker_data_type _Mymarkers;
	};

	class UncodedMarkers : public Markers
	{
		using _Mybase = Markers;
		using _Myt = UncodedMarkers;

	public:
		const marker_data_type &generate() noexcept override;
		QPixmap to_image() const noexcept override;
		void to_pdf() const noexcept override;
	};

	template <>
	class OpticalMarkers<om_marker_tag::CODED>
	{
		template <typename T = size_t>
		static constexpr auto _Mydpi = T(254);
		using _Mypie_type = om_code_sector;

	public:
		using value_t = float;
		using shared_qimage = std::shared_ptr<QImage>;
		using shared_pdfeng = std::shared_ptr<QPrinter>;
		using image_pointer = std::tuple<u_char *, size_t, size_t>;
		using marker_desc_cont = std::vector<om_marker_desc<value_t>>;
		struct options_type
		{
			value_t inner_radius = 2.5; // mm
			size_t start_id = 0;
			size_t end_id = 53;
			size_t code_bits = 15;
			value_t spacing = 0.5; // mm

			value_t min_radius{5};	// pix, Minimum inner radius of markers to be detected
			value_t max_radius{50}; // pix, Maximum inner radius of markers to be detected
			size_t segmentation{1}; // segmentation level with available values of 0, 1, 2, and 3
			bool enable_hf = false;
		};
		using options_t = options_type;

		/// <summary>
		/// CTOR, default
		/// </summary>
		OpticalMarkers() = default;

		/// <summary>
		/// \brief Generate coded makers according to given settings.
		/// </summary>
		const marker_data_type &encode(options_type options) noexcept;

		/// <summary>
		/// \brief Save the generated markers to QImage.
		/// </summary>
		/// <returns>Shared pointer to QImage</returns>
		QPixmap to_image() noexcept;

		/// <summary>
		/// \brief Export the generated makers to PDF file engine.
		/// </summary>
		/// <returns></returns>
		void to_pdf() noexcept;

		/// <summary>
		/// \brief Decode the coded makers.
		/// </summary>
		/// <returns>A vector holding coded markers.</returns>
		marker_desc_cont decode(const image_pointer image) noexcept;

		/// <summary>
		/// \brief Return the coded marker points detected by calling decode().
		/// </summary>
		const marker_desc_cont &get_coded_markers() const noexcept;
		/// <summary>
		/// \brief Return uncoded marker points detected by calling decode().
		/// </summary>
		const marker_desc_cont &get_uncoded_markers() const noexcept;

		/// <summary>
		/// \brief Get reserved markers. Default is the markers 0, 1, 2, and 3.
		/// This method is often used to seperate the codes on the scale-bar(s).
		/// </summary>
		/// <param name="markers">All detected markers by the method "decode". It will be overwritten by non-reserved ones.</param>
		/// <returns>Reserved marker list.</returns>
		marker_desc_cont get_reserved(marker_desc_cont &markers) noexcept;

		/// <summary>
		/// \brief Get the rect size of a single marker.
		/// </summary>
		/// <returns></returns>
		size_t marker_size() const noexcept;

		/// <summary>
		/// \brief SETTER/GETTER of marker options.
		/// </summary>
		decltype(auto) options() const noexcept
		{
			return (_Myoptions);
		}
		decltype(auto) options() noexcept
		{
			return (_Myoptions);
		}

		/// <summary>
		/// \brief SETTER to collect the info message.
		/// </summary>
		decltype(auto) message() const noexcept
		{
			return (_Mymsg);
		}

	private:
		size_t _Mysize;
		options_t _Myoptions;
		marker_data_type _Mydata;
		QString _Mymsg;

		marker_desc_cont _Myuncoded; // Holds uncoded marker points
		marker_desc_cont _Mycoded;	 // Holds coded marker points
	};
	using coded_optical_marker_t = OpticalMarkers<om_marker_tag::CODED>;

	/// <summary>
	/// \brief Convert the markers to image buffer.
	/// </summary>
	/// <param name="data">: \sa struct marker_data_type</param>
	/// <param name="idx">: index of the marker page</param>
	/// <returns>QPixmap showing a page of markers</returns>
	QPixmap forward_to_image(const marker_data_type &data, size_t idx = 0) noexcept;

	/// <summary>
	/// \brief Save the markers to a pdf file, with a pop-up file dialog for selecting target folder.
	/// </summary>
	/// <param name="data">: struct holds the marker data</param>
	/// <param name="cr">: center radius of the markers</param>
	void forward_to_pdf(const marker_data_type &data, float cr) noexcept;
}