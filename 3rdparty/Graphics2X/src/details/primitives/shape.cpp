#include "utils/editablegraphicsscene.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"
#include "primitives/shape.h"

void ShapePainter::exec()
{
	m_shape = new AreaItem();
	m_scene->addItem(m_shape);
	QPointF _Pt(0.0, 0.0);
	QPointF ref_point= _pList.value(0);
	QPolygonF poly = m_shape->polygon();
	for (int i = 0; i < _pList.length(); i++)
	{
		_Pt = _pList.value(i);
		if (_Pt.x() == 0 && _Pt.y() == 0)
			break;

		NodeItem* node = new NodeItem(m_shape);
		node->setPos(_Pt);
		m_scene->addItem(node);
		m_shape->node_list << node;
		poly << node->scenePos() - ref_point;
	}

	m_shape->setPos(ref_point);
	m_shape->setPolygon(poly);
	m_shape->setVisible(true);
	m_shape->SetNodesVisible(true);
	m_scene->m_shapes<<m_shape;

}

void ShapePainter::undo()
{
	m_scene->removeItem(m_shape);
	for(int i = 0; i<m_shape->node_list.length() ;i++)
	{
		m_scene->removeItem(m_shape->node_list.value(i));
		delete m_shape->node_list.value(i);

	}
	delete m_shape;
	m_shape = nullptr;

	m_scene->m_shapes.removeOne(m_shape);
}
