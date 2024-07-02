#pragma once
#include "utils/shapeevents.h"
#include <QList>

class ShapePainter : public ShapeEvent
{
public:
	ShapePainter(EditableGraphicsScene* scene, QList<QPointF> pList)
		: m_scene(scene),_pList(pList) {
	}

	void exec();
	void undo();

private:
	QList<QPointF> _pList;
	AreaItem* m_shape = nullptr;
	EditableGraphicsScene* m_scene;
};
