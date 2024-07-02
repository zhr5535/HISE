#include "primitives/rect.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"

/* initialize drawing rectangular */
void RectPainter::exec()
{
	/* each area node invisible */
	foreach (AreaItem* area, m_scene->m_shapes)
		area->SetNodesVisible(false);

	/* initialize the area */
	m_shape = new AreaItem();
	m_shape->setPolygon(QPolygonF(QRectF(QPointF(0,0),QPointF(0,0))));
	m_scene->addItem(m_shape);
	m_shape->setPos(topLeft);
	m_scene->m_shapes << m_shape;

	/* initialize the nodes of the area */
	for (int i = 0; i < 4; i++) {
		NodeItem* node = new NodeItem(m_shape);
		m_shape->node_list << node;
		m_scene->addItem(node);
		node->setPos(topLeft);
	}	

	/* set the area and nodes visible */
	m_shape->setVisible(true);
	m_shape->SetNodesVisible(true);
	m_scene->add_cursor_label();     // show the coordinate when mouse pressed

}
/* when drawing the rectangle, the rectangle move as the second point move */
void RectPainter::move()
{
	if (!m_scene->m_shapes.isEmpty()) {
		QRectF rect(topLeft,rightButtom);

		/* update the rectangle area */
		m_shape = m_scene->m_shapes.last();
		m_shape->setPolygon(QPolygonF(rect.translated(-topLeft)));

		/* update the node position */
		m_shape->node_list.value(0)->setPos(rect.topLeft());
		m_shape->node_list.value(1)->setPos(rect.topRight());
		m_shape->node_list.value(2)->setPos(rect.bottomRight());
		m_shape->node_list.value(3)->setPos(rect.bottomLeft());

		if (m_scene->m_is_merge_poly){
			m_scene->m_selectedregions = m_scene->m_shapes.length() - 1;
			m_scene->AddInitSet();
			m_scene->AddRegionMove(m_shape);
		}
		m_scene->move_cursor_label(rightButtom);  // move the coordinate label
	}
}
/* the mouse pressed, end drawing rectangular */
void RectPainter::merge()
{
	m_shape = m_scene->m_shapes.last();
	m_shape->SetNodesVisible(true);

	if (m_scene->m_is_merge_poly){
		m_scene->AddInitSet();
		m_scene->AddRegionPress(m_shape);
	}
	m_scene->delete_cursor_label();
}

void RectPainter::undo()
{
	m_scene->removeItem(m_shape);
	for(int i = 0; i<m_shape->node_list.length() ;i++)
		m_scene->removeItem(m_shape->node_list.value(i));
	m_scene->m_shapes.removeOne(m_shape);
}

void RectPainter::redo()
{
	m_scene->addItem(m_shape);
	for(int i = 0; i<m_shape->node_list.length() ;i++)
		m_scene->addItem(m_shape->node_list.value(i));
	m_scene->m_shapes<<m_shape;
}
