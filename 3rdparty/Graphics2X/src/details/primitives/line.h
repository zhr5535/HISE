#pragma once
#include <QPointF>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

#include "utils/shapeevents.h"
#include "utils/nodeitem.h"

class LineItem : public QGraphicsLineItem {
	using _Mybase = QGraphicsLineItem;
public:
	LineItem(QGraphicsLineItem* parent = nullptr)
		:_Mybase(parent) {
	}

	QList<QPointF> node_list;
protected:
	void mousePressEvent(QGraphicsSceneMouseEvent* e) {
		update();
		_Mybase::mousePressEvent(e);
	}
	void mouseMoveEvent(QGraphicsSceneMouseEvent* e) {
		update();
		if (e->buttons() & Qt::LeftButton) {
			decltype(auto) _Rect = scene()->sceneRect();
			decltype(auto) _Spos = e->scenePos();
			const auto _Max_x = _Rect.x() + _Rect.width();
			const auto _Max_y = _Rect.y() + _Rect.height();
			using std::min, std::max;
			const auto x = min(_Max_x, max(_Spos.x(), _Rect.x()));
			const auto y = min(_Max_y, max(_Spos.y(), _Rect.y()));
			setPos(x, y);
		}
		e->ignore();
		_Mybase::mouseMoveEvent(e);
	}
	void mouseReleaseEvent(QGraphicsSceneMouseEvent* e) {
		_Mybase::mouseReleaseEvent(e);
	}
};

class LinePainter : public ShapeEvent
{
public:
	LinePainter(EditableGraphicsScene* scene, QPointF point1)
		: m_scene(scene), _Mystartpos(point1) {
	}

	void exec() override;
	void undo() override;
	void move() override;
	void merge() override;
	void redo() override;

	void next();

private:
	QPointF _Mystartpos;
	LineItem* _Myline{ nullptr };
	EditableGraphicsScene* m_scene;
};

void LinePainter::exec()
{
	auto _Start = new NodeItem(nullptr);
}

inline void LinePainter::undo()
{
}

inline void LinePainter::move()
{
}

inline void LinePainter::merge()
{
}

inline void LinePainter::next()
{
}

inline void LinePainter::redo()
{
}
