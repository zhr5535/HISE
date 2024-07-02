#include "primitives/polygon.h"
#include "utils/regionitem.h"
#include "utils/nodeitem.h"
#include "utils/editablegraphicsscene.h"

/* initialize drawing polygon */
void PolygonPainter::exec()
{
	/* first make each node invisible */
	foreach (AreaItem* area, m_scene->m_shapes)
		area->SetNodesVisible(false);

	/* initialize the area */
	QPolygonF polygon;
	polygon << QPointF(0,0) << QPointF(0,0);
	_polygon_area = new AreaItem();
	_polygon_area->setPolygon(polygon);
	m_scene->addItem(_polygon_area);
	_polygon_area->setPos(_p1);
	m_scene->m_shapes << _polygon_area;

	/* initialize the nodes of the area */
	NodeItem* node = new NodeItem(_polygon_area);
	NodeItem* node_next = new NodeItem(_polygon_area);
	_polygon_area->node_list << node << node_next;
	m_scene->addItem(node);
	node->setPos(_p1);
	m_scene->addItem(node_next);
	node_next->setPos(_p1);

	/* make the polygon visible */
	_polygon_area->setVisible(true);
	_polygon_area->SetNodesVisible(true);
	m_scene->add_cursor_label();

}

void PolygonPainter::next()
/* continue drawing the polygon */
{
	_polygon_area = m_scene->m_shapes.last();

	/* add new node */
	NodeItem* node_next = new NodeItem(_polygon_area); 
	node_next->setPos(_p1);
	_polygon_area->node_list << node_next;
	m_scene->addItem(node_next);

	/* update the polygon of the areaitem */
	QPolygonF polygon = _polygon_area->polygon();
	polygon << polygon.last();
	_polygon_area->setPolygon(polygon);
}

void PolygonPainter::undo()
{
	m_scene->removeItem(_polygon_area);
	for(int i = 0; i<_polygon_area->node_list.length() ;i++)
	{
		m_scene->removeItem(_polygon_area->node_list.value(i));
	}
	m_scene->m_shapes.removeOne(_polygon_area);
}

void PolygonPainter::redo()
{
	m_scene->addItem(_polygon_area);
	for(int i = 0; i<_polygon_area->node_list.length() ;i++)
	{
		m_scene->addItem(_polygon_area->node_list.value(i));
	}
	m_scene->m_shapes<<_polygon_area;
}

void PolygonPainter::move()
{
	if (m_scene->m_shapes.isEmpty()) return;
	_polygon_area = m_scene->m_shapes.last();

	QPointF pos_refer = _polygon_area->node_list.value(0)->scenePos();
	NodeItem* node_next = _polygon_area->node_list.last();
	node_next->setPos(_p1);
	_polygon_area->setPos(pos_refer);
	QPolygonF poly = _polygon_area->polygon();
	poly.pop_back();
	poly << node_next->scenePos() - pos_refer;
	_polygon_area->setPolygon(poly);

	if (!m_scene->m_is_merge_poly) return;

	m_scene->m_selectedregions = m_scene->m_shapes.length()-1;
	m_scene->AddInitSet();
	m_scene->AddRegionMove(_polygon_area);
	m_scene->move_cursor_label(_p1);
}


void PolygonPainter::merge()
{
	if(m_scene->m_shapes.isEmpty()) return;

	_polygon_area = m_scene->m_shapes.last();

	if (_polygon_area->node_list.length()<3)
	{
		m_scene->DelArea(_polygon_area);
		m_scene->m_shapes.removeOne(_polygon_area);
		return;
	}

	m_scene->delete_cursor_label();

 	if (!m_scene->m_is_merge_poly) return;
 	m_scene->AddInitSet();
 	m_scene->AddRegionPress(_polygon_area);
}