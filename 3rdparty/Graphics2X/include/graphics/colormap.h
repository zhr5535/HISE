#pragma once
#include <QWidget>
#include <cmath>
#include "internal/configs.h"

NAMESPACE_BEGIN(dgelom)
class ScaleBar : public QWidget {
	Q_OBJECT
	using _Mybase = QWidget;
public:
	explicit ScaleBar(QWidget* parent = nullptr);

	ScaleBar& set_length(size_t value) noexcept;
	ScaleBar& set_width(size_t value) noexcept;
	ScaleBar& set_levels(size_t value) noexcept;
	ScaleBar& set_font(QFont value) noexcept;
	ScaleBar& set_range(qreal min, qreal max) noexcept;
	ScaleBar& set_title(QString txt) noexcept;

	QColor color(qreal value) const noexcept;

	void update();
	void update(qreal min, qreal max, size_t levels = 6);
	/// <summary>
	/// \brief Move the ScaleBar instance to (x,y)
	/// </summary>
	/// <param name="x"> x coordinate</param>
	/// <param name="y"> y coordinate</param>
	void move(int32_t x, int32_t y);

protected:
	void paintEvent(QPaintEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;

private:
	/// <summary>
	/// \brief Build color table
	/// </summary>
	void _Build();
	/// <summary>
	/// \brief Resize the scale-bar without moving its pos
	/// </summary>
	void _Resize();

	QVector<QColor> _Mycolors;
	qreal _Mymin{ 0 }, _Mymax{ 1 };
	int32_t _Mylen{ 343 }, _Myw{ 15 }, _Mycy{ 171 };
	int32_t _Mylevels{ 6 };
	int32_t _Mytxtoff{ 20 };
	QFont _Myfont;
	QString _Mytitle{};
};

template<typename T>
inline auto short_rainbow(float f, float min = 0, float max = 1) noexcept {
	f = (f - min) / (max - min);
	f = (1 - f) / 0.25;
	const int x = std::floor(f);
	const auto y = std::floor(255 * (f - x));
	uint8_t r, g, b;
	switch (x)
	{
	case 0: r = 255, g = y, b = 0; break;
	case 1: r = 255 - y, g = 255, b = 0; break;
	case 2: r = 0, g = 255, b = y; break;
	case 3: r = 0, g = 255 - y, b = 255; break;
	case 4: r = 0, g = 0, b = 255; break;
	default:break;
	}
	return T(r, g, b);
}

template<typename T>
inline auto long_rainbow(float f, float min = 0, float max = 1) noexcept {
	f = (f - min) / (max - min);
	f = (1 - f) / 0.2;
	const int x = std::floor(f);
	const auto y = std::floor(255 * (f - x));
	uint8_t r, g, b;
	switch (x)
	{
	case 0: r = 255, g = y, b = 0; break;
	case 1: r = 255 - y, g = 255, b = 0; break;
	case 2: r = 0, g = 255, b = y; break;
	case 3: r = 0, g = 255 - y, b = 255; break;
	case 4: r = y, g = 0, b = 255; break;
	case 5: r = 255, g = 0, b = 255; break;
	default:break;
	}
	return T(r, g, b);
}
NAMESPACE_END(dgelom)