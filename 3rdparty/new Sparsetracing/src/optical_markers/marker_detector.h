#pragma once

#include <tuple>
#include <vector>
#include "utils.h"
#include "marker_fwd.h"

namespace dgelom
{

	/*@{*/ // Literal operators
	constexpr float operator""_pix(long double value) noexcept
	{
		return float(value);
	}
	/*@}*/

	class MarkerDetector
	{
	public:
		using value_t = float;
		using pixel_t = uint8_t;
		using pixel_ptr = DGE_USE_STD(add_pointer_t)<pixel_t>;

		// STRUCT, marker descriptor
		struct marker_desc_type : om_marker_desc<value_t>
		{
			/// <summary>
			/// \brief Eval the fitting error at each candidate edge point.
			/// </summary>
			/// <param name="x, y">: Coords of candidate edge point.</param>
			/// <returns>error</returns>
			value_t eval(value_t x, value_t y) const noexcept;
		};
		// ALIAS, container (std::vector) of marker decriptor
		using marker_desc_cont = DGE_USE_STD(vector)<om_marker_desc<value_t>>;

		// ENUM, defines threshold algorithms for marker segmentation.
		enum class threshold_type : uint8_t
		{
			SELF = 3,
			ITER = 2,
			CANY = 1,
			OTSU = 0,
			TRIA = 8,
			MEAN = 9,
			GAUS = 10
		};

		// STRUCT, packet of marker detection control parameters.
		struct options_t
		{
			using ratio_t = value_t;
			threshold_type seg_tag{threshold_type::ITER};
			value_t max_radius{50.0_pix};
			value_t min_radius{5.0_pix};
			value_t quality{0.4_pix}; // lower value means higher quality
			value_t noise_level{1.0};
			ratio_t k[3]{2.f, 2.4f, 4.f};
			uint8_t forcolor{0}; // 0 for white and 1 for black
			uint8_t enhanced{false};
		};

		/// <summary>
		/// \brief CTOR, create a detector with rows and cols of marker image(s).
		/// </summary>
		/// <param name="rows">: Rows of the image(s) to be detected.</param>
		/// <param name="cols">: Cols of the image(s) to be detected.</param>
		MarkerDetector(size_t rows, size_t cols) noexcept;

		/// <summary>
		/// \brief GETER/SETTER, access marker detection parameters.
		/// </summary>
		const options_t &options() const noexcept;
		options_t &options() noexcept;

		/// <summary>
		/// \brief METHOD, perform marker detection. It is thread-safe in image level.
		/// </summary>
		/// <param name="data">: Pointer to the marker image data.</param>
		/// <returns>(std::)vector of the marker discriptors.</returns>
		marker_desc_cont detect(const pixel_ptr data) const noexcept;
		marker_desc_cont exect(const pixel_ptr data) const;

		/// <summary>
		/// \brief FUNC, check if a marker meets the expected area size.
		/// </summary>
		/// <param name="options"></param>
		/// <param name="w">: Width of the detected marker area.</param>
		/// <param name="h">: Height of the detected marker area.</param>
		/// <returns>BOOL, true for pass and false for reject.</returns>
		friend bool is_valid(marker_desc_type desc, options_t options) noexcept;

	private:
		size_t _Myimcols, _Myimrows;
		options_t _Myoptions;
	};
}
