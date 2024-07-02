#include <QSpacerItem>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QFocusEvent>
#include <QSettings>
#include <QLabel>
#include <QSlider>
#include "ui_scalview2d.h"
#include "graphics/colormap.h"
#include "graphics/views/scalview2d.h"

ScalableGraphicsView::ScalableGraphicsView(QWidget *parent)
	: QGraphicsView(parent), m_zoomslider(new QSlider()),
	plbTip(new QLabel()), plb_1(new QLabel()), plb_2(new QLabel()),
	m_imageinfolabel(new QLabel()), plbMean(new QLabel()),
	pbtnAZoom(new QPushButton), pbtnFocusROI(new QPushButton),
	pbtnFocusOn(new QPushButton), pbtnZoomIn(new QPushButton),
	pbtnZoomOut(new QPushButton)
{
	ui->setupUi(this);
	setStyleSheet("QLabel {background : rgba(224,224,224, 127);}");

	// initialization of controls
	plbTip->setVisible(false);
	plbMean->setVisible(false);
	m_imageinfolabel->setVisible(false);
	pbtnFocusOn->setVisible(false);
	pbtnFocusROI->setVisible(false);

	pbtnAZoom->setFixedSize(20, 20);
	pbtnAZoom->setProperty("ScalableGraphicsView", true);
	pbtnAZoom->setText("A");

	pbtnZoomIn->setFixedSize(20, 20);
	pbtnZoomIn->setProperty("ScalableGraphicsView", true);
	pbtnZoomOut->setFixedSize(20, 20);
	pbtnZoomOut->setProperty("ScalableGraphicsView", true);
	pbtnFocusOn->setFixedSize(20, 20);
	pbtnFocusOn->setProperty("ScalableGraphicsView", true);
	pbtnFocusROI->setFixedSize(20, 20);
	pbtnFocusROI->setProperty("ScalableGraphicsView", true);

	m_zoomslider->setOrientation(Qt::Horizontal);
	m_zoomslider->setRange(0, max_zoom_v);
	m_zoomslider->setValue(default_zoom_v);
	m_zoomslider->setPageStep(1);
	m_zoomslider->setFixedSize(100, 16);
	m_zoomslider->setFocusPolicy(Qt::NoFocus);
	m_zoomslider->setVisible(true);

	plb_1->setFixedSize(100, 2);
	plb_2->setFixedSize(100, 2);
	m_imageinfolabel->setAlignment(Qt::AlignCenter);

	///< Layout for widgets
	auto pVBLayout = new QVBoxLayout();
	//pVBLayout->addWidget(plb_1);
	pVBLayout->addWidget(m_zoomslider);
	//pVBLayout->addWidget(plb_2);
	auto pHSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
	auto pHBLayout = new QHBoxLayout();
	/*pHBLayout->addWidget(plbTip);
	pHBLayout->addWidget(pbtnZoomIn);
	pHBLayout->addLayout(pVBLayout);
	pHBLayout->addWidget(pbtnZoomOut);*/
	pHBLayout->addSpacerItem(pHSpacer);
	pHBLayout->addWidget(pbtnAZoom);
	pbtnAZoom->setStyleSheet("background-color:rgba(240,240,240,0)");
	pbtnAZoom->setIcon(QPixmap(""));
	/*pHBLayout->addWidget(pbtnFocusROI);
	pHBLayout->addWidget(pbtnFocusSubset);
	pHBLayout->addWidget(plbMean);*/
	pHBLayout->setSpacing(0);
	auto pVSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
	auto pVLayout = new QVBoxLayout(this);
	pVLayout->addLayout(pHBLayout);
	pVLayout->addSpacerItem(pVSpacer);
	pVLayout->addWidget(m_imageinfolabel);
	pVLayout->setContentsMargins(0, 0, 0, 0);

	this->viewport()->setMouseTracking(true);

	QFont font;
	font.setFamily(QString::fromUtf8("Arial"));
	setFont(font);
	m_imageinfolabel->setFont(font);
	m_imageinfolabel->setAlignment(Qt::AlignLeft);

	///< Scale bar, hidden in default
	m_scalebar = new dgelom::ScaleBar(this);
	m_scalebar->setVisible(false);

	///< Connection of signals and slots 
	connect(pbtnAZoom, &QPushButton::clicked, [&] {autofit(); });
	connect(pbtnZoomIn, &QPushButton::clicked, [&] {slotZoomIn(); });
	connect(pbtnZoomOut, &QPushButton::clicked, [&] {slotZoomOut(); });
	connect(m_zoomslider, &QSlider::valueChanged, [&](auto v) {slotZoomChanged(v); });
	connect(this, &ScalableGraphicsView::signal_resized, [&] { m_scalebar->move(geometry().right(), geometry().center().y()); });
}

ScalableGraphicsView::~ScalableGraphicsView() {}

#pragma region Public Methods
void ScalableGraphicsView::show_tip(const QString &tip) noexcept
{
	plbTip->setVisible(true);
	plbTip->setText(tip);
}

void ScalableGraphicsView::show_mean(const QString &mean) noexcept
{
	plbMean->setVisible(true);
	plbMean->setText(mean);
}

void ScalableGraphicsView::show_image_info(const QString &name) noexcept
{
	m_imageinfolabel->setVisible(true);
	m_imageinfolabel->setText(name);
}

void ScalableGraphicsView::set_ctrls_visible(bool is_visible) noexcept
{
	pbtnAZoom->setVisible(is_visible);
	pbtnZoomIn->setVisible(is_visible);
	pbtnZoomOut->setVisible(is_visible);
	m_zoomslider->setVisible(is_visible);
	plb_1->setVisible(is_visible);
	plb_2->setVisible(is_visible);
}

ScalableGraphicsView::scalebar_ptr ScalableGraphicsView::scalebar() const noexcept
{
	return m_scalebar;
}

QSlider* ScalableGraphicsView::zoom_slider() const noexcept
{
	return m_zoomslider;
}

void ScalableGraphicsView::focus_on(const QRect &rect) noexcept
{
	this->fitInView(rect, Qt::KeepAspectRatio);

	QTransform _Transf = this->transform();
	//this->setTransform(_Transf);
	const auto _Pre_scale = _Transf.m11();
	const auto _Cur_scale = this->transform().m11();
	const auto involution = log(_Cur_scale/_Pre_scale) / log(1.25);

	int count = involution;
	count = (involution - count)>0.5 ? count++ : count;
	count = (count - involution)>0.5 ? count-- : count;
	count += m_zoomslider->value();
	count < default_zoom_v ? count = default_zoom_v : count;
	m_zoomslider->setValue(count);
}
/* adaptive display according user input parameters if image resolution is low */
void ScalableGraphicsView::adaptive_ctrl(int _w, int _h) noexcept
{
	setSceneRect(0, 0, _w, _h);
	autofit();
}
#pragma endregion

#pragma region Slots
//fit in button, set the value of slider to 15
void ScalableGraphicsView::autofit() noexcept
{
	slotZoomChanged(default_zoom_v);
}

void ScalableGraphicsView::slot_set_cursor_shape(Qt::CursorShape shape) noexcept
{
	this->viewport()->setCursor(shape);
	this->viewportTransform();
}

void ScalableGraphicsView::slotZoomChanged(int val) noexcept
{
	pbtnZoomIn->setEnabled(true);
	pbtnZoomOut->setEnabled(true);

	if (val == default_zoom_v) {
		this->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
		m_zoomslider->setValue(val);
	}
	else {
		if (val > m_prezoomv) {
			for (int i = 0; i < val - m_prezoomv; i++)
				this->scale(1.15, 1.15);
			if (val == max_zoom_v)
				pbtnZoomOut->setEnabled(false);
		}
		else if (val < m_prezoomv) {
			for (int i = 0; i < m_prezoomv - val; i++)
				this->scale(0.85, 0.85);
			if (val == 0)
				pbtnZoomIn->setEnabled(false);
		}
	}
	m_prezoomv = val;
	emit signal_zoom_changed();
}

//zoom in by decreasing the value of slider by 1 
void ScalableGraphicsView::slotZoomIn() noexcept
{
	m_zoomslider->setValue(m_zoomslider->value() - 1);
}

//zoom out by increasing the value of slider by 1 
void ScalableGraphicsView::slotZoomOut() noexcept
{
	m_zoomslider->setValue(m_zoomslider->value() + 1);
}

void ScalableGraphicsView::slotMenuZoomIn() noexcept
{
	if (hasFocus()) slotZoomOut();
}
void ScalableGraphicsView::slotMenuZoomOut() noexcept
{
	if (hasFocus()) slotZoomIn();
}
void ScalableGraphicsView::slotMenuResume() noexcept
{
	if (hasFocus()) autofit();
}

#pragma endregion

#pragma region Events
void ScalableGraphicsView::resizeEvent(QResizeEvent *resizeEvent)
{
	this->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
	m_zoomslider->setValue(default_zoom_v);
	emit signal_resized();
	QGraphicsView::resizeEvent(resizeEvent);
}

//Plus 1 if wheel forward, else minus 1 
void ScalableGraphicsView::wheelEvent(QWheelEvent *event)
{
	event->angleDelta().y() > 0 ? slotZoomOut() : slotZoomIn();
}

void ScalableGraphicsView::mouseDoubleClickEvent(QMouseEvent* event)
{
	emit signal_double_click();
}

void ScalableGraphicsView::keyPressEvent(QKeyEvent * event)
{
	const auto key_value = event->key();
	switch (event->key()) {
	case Qt::Key_Equal:      // ctrl+  zoom in
		if (event->modifiers() & Qt::ControlModifier)
			slotMenuZoomOut();
		break;
	case Qt::Key_Minus:		// ctrl-  zoom out
		if (event->modifiers() & Qt::ControlModifier)
			slotMenuZoomIn();
		break;
	case Qt::Key_0:         // ctrl0  resume
		if (event->modifiers() & Qt::ControlModifier)
			autofit();
	default:
		QWidget::keyPressEvent(event);
	}
}

void ScalableGraphicsView::focusOnEvent(QFocusEvent* event)
{
	emit signal_focus_on();
}

void ScalableGraphicsView::changeEvent(QEvent *e)
{
	QWidget::changeEvent(e);
}
#pragma endregion
