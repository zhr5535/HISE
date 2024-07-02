#pragma once
#include "shapeclippingevents.h"
#include "utils/editablegraphicsscene.h"

class PolygonClipper : public ShapeClippingEvents
{
public:
	PolygonClipper(EditableGraphicsScene* scene, QPointF point1) 
		: m_scene(scene), _p1(point1){}

	void exec();
	void undo();
	void move();
	void clip();
	void next();

private:
	QPointF _p1;
	AreaItem* _polygon_area;

	EditableGraphicsScene* m_scene;
};
