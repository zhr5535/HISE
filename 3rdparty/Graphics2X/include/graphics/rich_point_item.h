#pragma once

#include <QMap>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include "internal/configs.h"

class AreaItem;
class QGraphicsScene;
class QGraphicsTextItem;
class QGraphicsSceneMouseEvent;

NAMESPACE_BEGIN(dgelom)
class RichPointItem : public QObject, public QGraphicsItem
{
	Q_OBJECT;
	using _Myt = RichPointItem;
public:
	explicit RichPointItem(QGraphicsScene* scene=nullptr);

	_Myt& set_movable(bool flag=true) noexcept;
	_Myt& set_seletable(bool flag=true) noexcept;
	_Myt& set_label(QString txt) noexcept;
	_Myt& set_label(size_t number) noexcept;
	
	void select(bool flag) noexcept;
	void remove(bool flag = true) noexcept;

	const QGraphicsTextItem* label() const noexcept;
	QGraphicsTextItem* label() noexcept;

	void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
	QRectF boundingRect() const;

signals:
	void moved(const QPointF&);
	void cursor_shape(Qt::CursorShape);
	void show_prop();
	void deleted(uint32_t); // Pass the index of the deleted point

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent*);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent*);
	void mouseMoveEvent(QGraphicsSceneMouseEvent*);

private:
	bool _Mydragend{ true };

	QMap<QString, float> _Mydict; // Dict map for holding properties
	QColor _Mycolors[3];
	QGraphicsScene* _Myscene{ nullptr };
	QGraphicsTextItem _Mylabel{ nullptr };
};
NAMESPACE_END(dgelom)