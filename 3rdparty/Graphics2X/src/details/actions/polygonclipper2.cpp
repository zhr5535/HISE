#include "actions/polygonclipper2.h"
#include "utils/nodeitem.h"
#include <QList>
#include <QPointF>


void PolygonCutCommand::exec()
{

}

void PolygonCutCommand::undo()
{

}

void PolygonCutCommand::ClipMove()
/* when the mouse move, clip the region */
{
	if(movingArea->node_list.length() <= 2)
		return;

	QVector<AreaItem*> fixedAreas = m_scene->AreaToMulitArea(fixedArea);   // change the fixed area to a list of areas
	AreaItem* fixedAreaMain = fixedAreas.first();

	bool _isContain  = m_scene->AreaContainArea(fixedAreaMain,movingArea); // whether the fixedArea contain movingArea
	if (_isContain)  // if the fix area contain cutting area
	{
		QList<QPointF> resultMainPointList,resultClipPointList;
		resultMainPointList = m_scene->AreaToPointList(fixedAreaMain);

		bool innerAreaContainMovingArea = false; // whether the inner areas contain cutting area
		for(int i = 1; i < fixedAreas.count(); i++)
		{
			bool movingAreaContainInnerArea  = m_scene->AreaContainArea(movingArea,fixedAreas[i]);  // whether the cutting area contain the inner areas of fixedAreas
			if (!movingAreaContainInnerArea) // moving area don't contain inner area i
			{
				AreaItem* fixedAreasInnerAreaCopy = m_scene->CopyArea(fixedAreas[i]);
				AreaItem* movingAreaCopy = m_scene->CopyArea(movingArea);//
				QList<QPointF> tempAddResultPointList = m_scene->AddAnyRegion(movingAreaCopy,fixedAreasInnerAreaCopy,false);

				if (tempAddResultPointList.isEmpty()) // movingArea don't intersect with inner area i or inner area i contain movingArea
					resultClipPointList << m_scene->AreaToPointList(fixedAreas[i]) << QPointF(-1.0,-1.0);
				else
					movingArea = m_scene->PListToNoAddArea(tempAddResultPointList);
			}

			if (m_scene->AreaContainArea(fixedAreas[i],movingArea))
				innerAreaContainMovingArea = true;
		}

		if (!innerAreaContainMovingArea)
			resultClipPointList << m_scene->AreaToPointList(movingArea) << QPointF(-1.0,-1.0);

		resultMainPointList << QPointF(-1.0,-1.0) << resultClipPointList;

		AreaItem* _areaCliped = m_scene->PListToArea(resultMainPointList);
		_areaCliped->item_select =true;
		areaArray << _areaCliped;

		fixedArea->setVisible(false);
		fixedArea->SetNodesVisible(false);
		fixedArea->item_select = true;
		movingArea->pen_style = true;
		_isMerged = true;

		return;
	}

	//用外围区域与剪裁区域相剪裁
	AreaItem* movingAreaCopy = m_scene->CopyArea(movingArea);
	QList<QPointF> boundaryClipResultPointList = m_scene->ClipAnyRegion(fixedAreaMain, movingAreaCopy, false);
	if(boundaryClipResultPointList.isEmpty())
		return;

	//得到剪裁后所生成的区域数组
	QVector<AreaItem*> _areaArray = m_scene->PListToMulitArea(boundaryClipResultPointList);

	QList<QPointF > noInnerAreaResultPointList;
	QList<QPointF > innerAreaResultMainPointList, innerAreaResultMainClipList;
	for(int num = 0; num < _areaArray.count(); num++)
	{
		noInnerAreaResultPointList.clear();
		innerAreaResultMainPointList.clear();
		innerAreaResultMainClipList.clear();

		if (fixedAreas.count() == 1) // there is no inner area
		{
			noInnerAreaResultPointList << m_scene->AreaToPointList(_areaArray[num]);
			AreaItem* area = m_scene->PListToArea(noInnerAreaResultPointList);  // add area to result directly
			area->item_select =true;
			areaArray << area;
		}
		else  // there is inner area
		{
			for (int i = 1; i < fixedAreas.count(); i++)
			{
				bool movingAreaContainInnerArea  = m_scene->AreaContainArea(movingArea,fixedAreas[i]);
				if (movingAreaContainInnerArea) // if moving area contain inner area
				{
					innerAreaResultMainPointList = m_scene->AreaToPointList(_areaArray[num]);
					continue;
				}
				else // if moving area do not contain inner area
				{
					AreaItem* fixedAreaClipResultArea = _areaArray[num];
					AreaItem* fixedAreaCopy = m_scene->CopyArea(fixedAreas[i]);
					QList<QPointF> tempClipResultPointList= m_scene->ClipAnyRegion(fixedAreaClipResultArea ,fixedAreaCopy, false);
					if (tempClipResultPointList.isEmpty()) // the inner area is  outside _areaArray[num]
						innerAreaResultMainPointList = m_scene->AreaToPointList(fixedAreaClipResultArea);
					else // the inner area in or intersect with _areaArray[num]
					{
						bool outerPolygon = true; // if outerPolygon is true, we add the point to mainPointList
						QList<QPointF> mainPointList;
						QList<QPointF> clipPointList;
						foreach (QPointF point, tempClipResultPointList)
							if (point == QPointF(-1.0,-1.0))
								outerPolygon = false;
							else if (outerPolygon)   // the point is a outer point
								mainPointList << point;
							else                     // the point is a inner point
								clipPointList << point;

						innerAreaResultMainPointList = mainPointList;

						_areaArray[num] = m_scene->PListToNoAddArea(innerAreaResultMainPointList);

						if (!clipPointList.isEmpty())
						{
							clipPointList << QPointF(-1.0,-1.0);
							innerAreaResultMainClipList.append(clipPointList);
						}
					}
				}
			}

			innerAreaResultMainPointList << QPointF(-1.0,-1.0);
			if (!innerAreaResultMainClipList.isEmpty())
				innerAreaResultMainPointList.append(innerAreaResultMainClipList);

			AreaItem* _areaCliped = new AreaItem();
			_areaCliped = m_scene->PListToArea(innerAreaResultMainPointList);
			_areaCliped->item_select =true;
			areaArray << _areaCliped;
		}
	}

	fixedArea->setVisible(false);
	fixedArea->SetNodesVisible(false);
	fixedArea->item_select = true;

	movingArea->pen_style = true;

	_isMerged = true;
}

void PolygonCutCommand::ClipDo()
{
	if(movingArea->node_list.length() <= 2)
		return;

	QVector<AreaItem* > fixedAreas = m_scene->AreaToAreas(fixedArea);
	AreaItem* fixedAreaMain = fixedAreas.first();

	bool _isContain  =m_scene->AreaContainArea(fixedAreaMain,movingArea); // whether the fixedArea contain movingArea
	if (_isContain)
	{
		QList<QPointF> resultMainPointList,resultClipPointList;
		resultMainPointList = m_scene->AreaToPointList(fixedAreaMain);

		bool innerAreaContainMovingArea = false; // whether the inner areas contain cutting area
		for(int i = 1; i < fixedAreas.count(); i++)
		{
			bool movingAreaContainInnerArea  =m_scene->AreaContainArea(movingArea,fixedAreas[i]);  // whether the cutting area contain the inner areas of fixedAreas
			if (!movingAreaContainInnerArea) // moving area don't contain inner area i
			{
				AreaItem* fixedAreasInnerAreaCopy = m_scene->CopyArea(fixedAreas[i]);
				AreaItem* movingAreaCopy = m_scene->CopyArea(movingArea);//
				QList<QPointF> tempAddResultPointList = m_scene->AddAnyRegion(movingAreaCopy,fixedAreasInnerAreaCopy,false);

				if (tempAddResultPointList.isEmpty())
					resultClipPointList << m_scene->AreaToPointList(fixedAreas[i]) << QPointF(-1.0,-1.0);
				else
					movingArea=m_scene->PListToNoAddArea(tempAddResultPointList);

				if (m_scene->AreaContainArea(fixedAreas[i],movingArea))
					innerAreaContainMovingArea = true;
			}
		}

		if (!innerAreaContainMovingArea)
			resultClipPointList << m_scene->AreaToPointList(movingArea) << QPointF(-1.0,-1.0);

		resultMainPointList << QPointF(-1.0,-1.0) << resultClipPointList;

		AreaItem* _areaCliped = m_scene->PListToArea(resultMainPointList);
		_areaCliped->item_select =true;
		m_scene->m_shapes << _areaCliped;

		m_scene->DelArea(fixedArea);
		m_scene->m_shapes.removeOne(fixedArea);
		_isMerged = true;
		return;
	}

	//用外围区域与剪裁区域相剪裁
	AreaItem* movingAreaCopy = m_scene->CopyArea(movingArea);
	QList<QPointF> boundaryClipResultPointList = m_scene->ClipAnyRegion(fixedAreaMain, movingAreaCopy, false);
	////说明：pList1,如果movingAreaCopy在fixedAreas[_areaIndex]内部则最后没有Qpoint(-1.0,-1.0)
	////如果只是相交则最后有Qpoint(-1.0,-1.0)
	if(boundaryClipResultPointList.isEmpty())
		return;

	//得到剪裁后所生成的区域数组
	QVector<AreaItem*> _areaArray = m_scene->PListToMulitArea(boundaryClipResultPointList);

	QList<QPointF > noInnerAreaResultPointList;
	QList<QPointF > innerAreaResultMainPointList, innerAreaResultMainClipList;
	for(int num = 0; num < _areaArray.count(); num++)
	{
		noInnerAreaResultPointList.clear();
		innerAreaResultMainPointList.clear();
		innerAreaResultMainClipList.clear();

		if (fixedAreas.count() == 1) // there is no inner area
		{
			noInnerAreaResultPointList << m_scene->AreaToPointList(_areaArray[num]);
			AreaItem* area = m_scene->PListToArea(noInnerAreaResultPointList);  // add area to result directly
			area->item_select =true;
			m_scene->m_shapes << area;
		}
		else  // there is inner area
		{
			for (int i = 1; i < fixedAreas.count(); i++)
			{
				bool movingAreaContainInnerArea  = m_scene->AreaContainArea(movingArea,fixedAreas[i]);
				if (movingAreaContainInnerArea) // if moving area contain inner area
				{
					innerAreaResultMainPointList = m_scene->AreaToPointList(_areaArray[num]);
					continue;
				}
				else // if moving area do not contain inner area
				{
					AreaItem* fixedAreaClipResultArea = _areaArray[num];
					AreaItem* fixedAreaCopy = m_scene->CopyArea(fixedAreas[i]);
					QList<QPointF> tempClipResultPointList= m_scene->ClipAnyRegion(fixedAreaClipResultArea ,fixedAreaCopy, false);
					if (tempClipResultPointList.isEmpty()) // the inner area is  outside _areaArray[num]
						innerAreaResultMainPointList = m_scene->AreaToPointList(fixedAreaClipResultArea);
					else // the inner area in or intersect with _areaArray[num]
					{
						bool outerPolygon = true; // if outerPolygon is true, we add the point to mainPointList
						QList<QPointF> mainPointList;
						QList<QPointF> clipPointList;
						foreach (QPointF point, tempClipResultPointList)
							if (point == QPointF(-1.0,-1.0))
								outerPolygon = false;
							else if (outerPolygon)   // the point is a outer point
								mainPointList << point;
							else                     // the point is a inner point
								clipPointList << point;

							innerAreaResultMainPointList = mainPointList;

							_areaArray[num] = m_scene->PListToNoAddArea(innerAreaResultMainPointList);

							if (!clipPointList.isEmpty())
							{
								clipPointList << QPointF(-1.0,-1.0);
								innerAreaResultMainClipList.append(clipPointList);
							}
					}
				}
			}

			innerAreaResultMainPointList << QPointF(-1.0,-1.0);
			if (!innerAreaResultMainClipList.isEmpty())
				innerAreaResultMainPointList.append(innerAreaResultMainClipList);

			AreaItem* _areaCliped = m_scene->PListToArea(innerAreaResultMainPointList);
			_areaCliped->item_select =true;
			m_scene->m_shapes << _areaCliped;
		}
	}

	m_scene->DelArea(fixedArea);
	m_scene->m_shapes.removeOne(fixedArea);
	_isMerged = true;

}