#pragma once

#include <QMenu>
#include <QPainter>
#include <QWheelEvent>
#include <QMessageBox>
#include "graphics/colormap.h"

NAMESPACE_BEGIN(dgelom)
static constexpr auto padding = 50;
ScaleBar::ScaleBar(QWidget* parent) 
	:_Mybase(parent) 
{
	_Myfont.setFamily(QString::fromUtf8("Arial"));
	_Myfont.setPointSize(10);

	_Build();
	
	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, &ScaleBar::customContextMenuRequested, [&](auto&& pos) {
		auto menu = QMenu();
		menu.addAction(tr("Hide"), [=] {setVisible(false); });
		menu.addSeparator();
		menu.addAction(tr("Level(+)"), [=] {_Mylevels++; update(); });
		menu.addAction(tr("Level(-)"), [=] {_Mylevels--; update(); });
		menu.addSeparator();
		menu.addAction(tr("Properties"), [=] {
			QMessageBox::information(this, tr("Deflow Message"), tr("Advanced settings are temporarily not allowed.")); });
		menu.exec(QCursor::pos());
		});
}

ScaleBar& ScaleBar::set_length(size_t length) noexcept 
{
	_Mylen = length;
	return (*this);
}

ScaleBar& ScaleBar::set_width(size_t value) noexcept
{
	_Myw = value;
	return (*this);
}

ScaleBar& ScaleBar::set_levels(size_t value) noexcept
{
	_Mylevels = value;
	return (*this);
}

ScaleBar& ScaleBar::set_font(QFont value) noexcept
{
	_Myfont = value;
	return (*this);
}

ScaleBar& ScaleBar::set_range(qreal min, qreal max) noexcept
{
	_Mymin = min, _Mymax = max;
	return (*this);
}

ScaleBar& ScaleBar::set_title(QString txt) noexcept
{
	_Mytitle = txt;
	return (*this);
}

QColor ScaleBar::color(qreal value) const noexcept
{
	if (value > _Mymax || value < _Mymin) {
		return { 0,0,0 };
	}
	const auto idx = (value - _Mymin) / (_Mymax - _Mymin) * _Mylen;
	return _Mycolors[idx];
}

void ScaleBar::update()
{
	_Build();
	_Mybase::update();
	if (!isVisible()) {
		setVisible(true);
	}
}

void ScaleBar::update(qreal min, qreal max, size_t levels)
{
	_Mymin = min, _Mymax = max;
	_Mylevels = levels;
	update();
}

void ScaleBar::move(int32_t x, int32_t y)
{
	_Mycy = y;
	x -= _Myw + padding;
	y = _Mycy - (_Mylen >> 1);
	setGeometry(QRect(x, y, _Myw + padding, _Mylen+padding));
	_Mybase::update();
}

void ScaleBar::paintEvent(QPaintEvent* event)
{
	if (height() < _Mylen) {
		return;
	}

	const auto enable_efmt = _Mymax < 0.001 || _Mymax > 1000;
	const auto fmt = enable_efmt ? 'E' : 'f';
	const auto wid = enable_efmt ? 2 : 4;
	QPainter painter(this);
	painter.setFont(_Myfont);
	for (auto i = 0; i < _Mylen; ++i) {
		painter.fillRect(QRect{ 0, _Mylen-i+_Myw+10, _Myw, 1 }, _Mycolors[i]);
	}
	const auto fm = QFontMetrics(_Myfont);
	for (auto i = 0; i < _Mylevels; ++i) {
		const auto val = _Mymin + i * (_Mymax - _Mymin) / (_Mylevels-1);
		const auto text = QString("%1").arg(val, 0, fmt, wid);
		const auto rect = fm.boundingRect(text);
		const auto tw = rect.width()+_Myw, th = rect.height();
		const auto pos = 10 + _Mylen - float_t(_Mylen)/(_Mylevels-1)*i;
		painter.drawText(QRect{ _Mytxtoff, int32_t(pos)+th/2, tw, th }, text);
	}
	auto title_font = _Myfont;
	title_font.setBold(true);
	painter.setFont(title_font);
	const auto rect = fm.boundingRect(_Mytitle);
	const auto dx = (geometry().width() - rect.width())>>1;
	painter.drawText(QRect{0, 0, rect.width(), rect.height()}, _Mytitle);
	_Mybase::paintEvent(event);
}
void ScaleBar::wheelEvent(QWheelEvent* event)
{
	const auto delta = event->angleDelta().y();
	const auto rate = delta / 1000.0 + 1;
	_Mylen *= rate;
	_Resize();
	_Mybase::wheelEvent(event);
}
void ScaleBar::_Build()
{
	_Mycolors.clear();
	for (auto l = 0; l < _Mylen; ++l) {
		const auto f = float_t(l) / _Mylen * (_Mymax - _Mymin) + _Mymin;
		_Mycolors.push_back(short_rainbow<QColor>(f, _Mymin, _Mymax));
	}
}
void ScaleBar::_Resize()
{
	const auto x = geometry().x();
	const auto y = _Mycy - (_Mylen>>1);
	setGeometry(x, y, _Myw + padding, _Mylen + padding*2);
	update();
}
NAMESPACE_END(dgelom)