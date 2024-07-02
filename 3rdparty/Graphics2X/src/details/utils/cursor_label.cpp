#include <QGraphicsItem>
#include <QPointF>
#include <QRectF>
#include <QPainter>
#include "utils/cursor_label.h"

CursorLabel::CursorLabel(const QString &text, QGraphicsItem *parent)
{
}

CursorLabel::~CursorLabel()
{
}

void CursorLabel::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
	painter->setBrush(QColor(131,225,131));
	painter->drawRect(boundingRect());
	painter->setRenderHint(QPainter::Antialiasing,true);

	QGraphicsTextItem::paint(painter,option,widget);
}
