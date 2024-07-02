#include <QGraphicsScene>
#include <QGraphicsView>
#include "utils/handcursorevents.h"
#include "utils/regionitem.h"
#include "nodeitem.h"
#include "pointitem.h"

void HandCursorEvent::press() noexcept
{
	/* select area */
	if(!m_scene->m_shapes.isEmpty()) {
		AreaItem* selectedArea = nullptr;  // select the selected area
		for (decltype(auto) area : m_scene->m_shapes) {
			if (area->PointInArea(area->mapFromScene(handPos)))
				selectedArea = area;
		}
		if (selectedArea != nullptr) { // set the node visible
			for (decltype(auto) area : m_scene->m_shapes) {
				area->setVisible(true);
				area->SetNodesVisible(false);
			}
			selectedArea->SetNodesVisible(true);
		}
	}

	/* show the coordinate when mouse press on the seed or node */
	if ((m_scene->m_translate_seed = m_scene->cursor_captured_point(handPos)) != nullptr ) { //press on the seed
		m_scene->m_is_seed_selected = true;
		m_scene->m_last_seed_pos = QPointF(m_scene->m_translate_seed->scenePos());
		m_scene->add_cursor_label();
	}
	else if (m_scene->cursor_captured_node(handPos) != nullptr) { // if press on the node
		m_scene->add_cursor_label();
	}

}

/* when hand on the node, the node become red */
void HandCursorEvent::move() noexcept
{
	for (decltype(auto) area : m_scene->m_shapes) {
		area->SetNodesSelect(handPos);
	}
}
