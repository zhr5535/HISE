/**********************************************************************
 * Copyright (C) 2021-2023, Zhilong (Dgelom) Su, All Rights Reserved.
 *
 * This software is distributed WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 *********************************************************************/
#pragma once
#include <QGraphicsView>
#include <QImage>

namespace Ui {class ScalableGraphicsView; }
namespace dgelom { class ScaleBar; }
class QLabel;
class QSlider;
class QPushButton;

class ScalableGraphicsView : public QGraphicsView
{
	Q_OBJECT
public:
	static constexpr auto max_zoom_v = 80;
	static constexpr auto default_zoom_v = 15;
	using scalebar_ptr = dgelom::ScaleBar*;

	ScalableGraphicsView(QWidget* parent = Q_NULLPTR);
	~ScalableGraphicsView();

	/* methods */
	void show_tip(const QString& tip) noexcept;
	void show_mean(const QString& mean) noexcept;
	void show_image_info(const QString& imgname) noexcept;
	void focus_on(const QRect& rect) noexcept;
	void adaptive_ctrl(int w, int h) noexcept;
	void set_ctrls_visible(bool flag) noexcept;

	// Get the pointer to the built ScaleBar
	scalebar_ptr scalebar() const noexcept;

	// Get the pointer to the built ZoomSlider
	QSlider *zoom_slider() const noexcept;

	/* signals */
signals:
	void signal_zoom_changed();
	void signal_double_click();
	void signal_focus_on();
	void signal_resized();

	/* slots */
public slots:
	// SLOT, autofit the view.
	void autofit() noexcept;
	void slot_set_cursor_shape(Qt::CursorShape shape) noexcept;

private slots:
	void slotZoomChanged(int val) noexcept;
	void slotZoomIn() noexcept;
	void slotZoomOut() noexcept;
	void slotMenuZoomIn() noexcept;
	void slotMenuZoomOut() noexcept;
	void slotMenuResume() noexcept;

	/* events */
private:
	void mouseDoubleClickEvent(QMouseEvent* event);
	void wheelEvent(QWheelEvent* event);
	void keyPressEvent(QKeyEvent* event);
	void focusOnEvent(QFocusEvent* event);
	void resizeEvent(QResizeEvent* event);

protected:
	void changeEvent(QEvent* e);

	/* fields */
private:
	Ui::ScalableGraphicsView* ui = nullptr;
	QPushButton* pbtnFocusOn = nullptr, * pbtnFocusROI = nullptr;
	QPushButton* pbtnAZoom = nullptr, * pbtnZoomIn = nullptr;
	QPushButton* pbtnZoomOut = nullptr;
	QLabel* plbTip = nullptr, * plbMean = nullptr;
	QLabel* m_imageinfolabel = nullptr;
	QLabel* plb_1 = nullptr, * plb_2 = nullptr;
	QSlider* m_zoomslider = nullptr;

	dgelom::ScaleBar* m_scalebar{ nullptr };

	size_t m_prezoomv = default_zoom_v;

	QImage m_image;
};

namespace dgelom {
template<bool Scalable = true>
inline auto make_graphicsview(QWidget* parent = nullptr) noexcept {
	if constexpr (Scalable) {
		return new ScalableGraphicsView(parent);
	}
	else {
		return new QGraphicsView(parent);
	}
}
}