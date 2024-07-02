#pragma once
#include <QPointF>
#include "utils/shapeevents.h"

class PolygonPainter : public ShapeEvent
{
public:
	PolygonPainter(EditableGraphicsScene* scene, QPointF point1) 
		: m_scene(scene), _p1(point1){
	}

	void exec();
	void undo();
	void move();
	void merge();
	void next();
	void redo();

private:
	QPointF _p1;
	AreaItem* _polygon_area{ nullptr };
	EditableGraphicsScene* m_scene;
};
 