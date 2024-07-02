#include <QMenu>
#include <QPainter>
#include <QGraphicsScene>
#include "graphics/rich_point_item.h"
#include "details/utils/nodeitem.h"
#include "details/utils/regionitem.h"

NAMESPACE_BEGIN(dgelom)
RichPointItem::RichPointItem(QGraphicsScene* scene)
	:QGraphicsItem(), _Myscene{ scene }
{
	setFlag(QGraphicsItem::ItemIsMovable, true);
	setFlag(QGraphicsItem::ItemIsSelectable, true);
	setFlag(QGraphicsItem::ItemIsFocusable, true);
	setFlag(QGraphicsItem::ItemIgnoresTransformations, true);

	_Mycolors[0] = QColor(239, 52, 60, 255);
	_Mycolors[1] = QColor(255, 255, 255, 200);
	_Mycolors[2] = QColor(235, 81, 81, 200);

	_Mylabel.setDefaultTextColor(_Mycolors[1]);
	_Mylabel.setFont(QFont("Arial", 16));
	_Mylabel.setVisible(true);
	_Mylabel.setFlag(QGraphicsItem::ItemIgnoresTransformations, true);

	_Myscene->addItem(this);
	_Myscene->addItem(label());
}
RichPointItem& RichPointItem::set_movable(bool flag) noexcept
{
	setFlag(QGraphicsItem::ItemIsMovable, flag);
	return (*this);
}
RichPointItem& RichPointItem::set_seletable(bool flag) noexcept
{
	setFlag(QGraphicsItem::ItemIsSelectable, flag);
	return (*this);
}
RichPointItem& RichPointItem::set_label(QString txt) noexcept
{
	_Mylabel.setPlainText(txt);
	update();
	return (*this);
}
RichPointItem& RichPointItem::set_label(size_t number) noexcept
{
	return set_label(QString::number(number));
}
void RichPointItem::select(bool flag) noexcept
{
	_Mydragend = flag;
	setSelected(flag);
	update();
}

void RichPointItem::remove(bool flag) noexcept
{
	_Myscene->removeItem(this);
	_Myscene->removeItem(&_Mylabel);
	this->setEnabled(!flag);
}

const QGraphicsTextItem* RichPointItem::label() const noexcept {
	return label();
}
QGraphicsTextItem* RichPointItem::label() noexcept {
	return std::addressof(_Mylabel);
}

void RichPointItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
	Q_UNUSED(option);
	Q_UNUSED(widget);
	setSelected(!_Mydragend);
	painter->setRenderHint(QPainter::Antialiasing, true);
	painter->setPen(_Mydragend?Qt::NoPen:QPen(Qt::green, 2));
	painter->setBrush(_Mydragend ? QBrush(_Mycolors[1]) : QBrush(_Mycolors[0]));
	painter->drawEllipse(QRectF(-5, -5, 10, 10));
	_Mylabel.setPos(pos() + QPointF(10, -25));
}

QRectF RichPointItem::boundingRect() const
{
	return QRectF(-5, -5, 10, 10);
}

void RichPointItem::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
	_Mydragend = false;
	if (event->button() & Qt::RightButton) {
		auto menu = QMenu();
		menu.addAction(tr("Show"), [=] { emit show_prop(); });
		menu.addAction(tr("Relabel"), [=] {});
		menu.addSeparator();
		menu.addAction(tr("Delete"), [=] {
			emit deleted(label()->toPlainText().toUInt()-1);
			_Myscene->removeItem(this);
			_Myscene->removeItem(&_Mylabel);
			this->setEnabled(false); });
		menu.exec(QCursor::pos());
	}
	update();
	QGraphicsItem::mousePressEvent(event);
}
void RichPointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
	_Mydragend = true;
	update();
	emit moved(this->pos());
	emit cursor_shape(Qt::CursorShape::PointingHandCursor);
	QGraphicsItem::mouseReleaseEvent(event);
}
void RichPointItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
	_Mydragend = false;
	update();
	if ((event->buttons() & Qt::LeftButton) && (flags() & ItemIsMovable)) {
		const auto srect = scene()->sceneRect();
		const auto max_x = srect.x() + srect.width();
		const auto max_y = srect.y() + srect.height();

		auto point = event->scenePos();
		if (point.x() < srect.x()) point.setX(srect.x());
		else if (point.x() > max_x) point.setX(max_x);
		else point.x();

		if (point.y() < srect.y()) point.setY(srect.y());
		else if (point.y() > max_y)point.setY(max_y);
		else point.y();

		setPos(point);
		event->ignore();
		//return;
	}
	else event->ignore();
	QGraphicsItem::mouseMoveEvent(event);
}
NAMESPACE_END(dgelom)