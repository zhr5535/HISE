#pragma once

#include "utils/editablegraphicsscene.h"
#include "utils/regionitem.h"

class PolygonMerger 
{
public:
	PolygonMerger(EditableGraphicsScene* scene, AreaItem* area1, AreaItem* area2):
	  m_scene(scene), fixedArea(area1), movingArea(area2), _areaMerged(nullptr), _isMerged(false) {}

	void exec();
	void undo();

	void MergeMove() ;
	void merge();


	bool IsMerged() {return _isMerged;}
	AreaItem* GetMergedArea() {return _areaMerged;}

private:

	EditableGraphicsScene* m_scene;

	AreaItem* fixedArea;
	AreaItem* movingArea;
	AreaItem* _areaMerged;

	bool _isMerged;

};
