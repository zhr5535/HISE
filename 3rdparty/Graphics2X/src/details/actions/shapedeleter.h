#pragma once

#include <QPointF>
#include "utils/editablegraphicsscene.h"

class DeleteAreaCommand
{
public:
	DeleteAreaCommand(EditableGraphicsScene* scene, QPointF point1) : m_scene(scene), _p1(point1) {}
	void exec();
	void undo();

private:
	QPointF _p1;
	EditableGraphicsScene* m_scene;
};
