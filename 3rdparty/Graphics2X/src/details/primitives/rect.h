#pragma once
#include <QPointF>
#include "utils/shapeevents.h"
#include "utils/editablegraphicsscene.h"

class RectPainter : public ShapeEvent
{
public:
	RectPainter(EditableGraphicsScene* scene, QPointF point1, QPointF point2) 
		: m_scene(scene), topLeft(point1), rightButtom(point2) {}
	void exec();
	void undo();
	void move();
	void merge();
 	void redo();

	
private:
	QPointF topLeft;
	QPointF rightButtom;

	AreaItem* m_shape;
	EditableGraphicsScene* m_scene;
};
