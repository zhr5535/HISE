#pragma once

#include <QGraphicsTextItem>

class QGraphicsItem;
class QStyleOptionGraphicsItem;
class QPainter;

/// <summary>
/// \brief Make a dynamic label to show the cursor position.
/// </summary>
class CursorLabel : public QGraphicsTextItem
{
public:
	CursorLabel(const QString &text, QGraphicsItem *parent = 0);
	~CursorLabel();

private:
	void paint(QPainter* painter, const QStyleOptionGraphicsItem *option, QWidget* widget);
};