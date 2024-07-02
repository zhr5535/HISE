#pragma once

#include <QGraphicsItem>

class SeedItem : public QGraphicsItem
{
public:
	SeedItem();
	~SeedItem();

	QRectF boundingRect() const;

	void paint(QPainter *painter, 
		const QStyleOptionGraphicsItem *option, 
		QWidget *widget);

	QPainterPath shape() const;
	QPainterPath* mutable_shape() const noexcept;

private:
	mutable QPainterPath _Mypath;
};

class PointItem : public SeedItem 
{
public:
	enum class type_tag : uint8_t
	{
		CROSS = 0,
		FCROSS = 1, //> Framed cross point
		FDOT = 2, //> Filled dot point
		HDOT = 3, //> Hollow dot point
		SEED = 4
	};
	using type_t = type_tag;

	PointItem(type_tag type=type_t::FCROSS) noexcept;

	void set_label(QString label) noexcept;
	void set_label(size_t id) noexcept;

	size_t label() const noexcept;

	friend bool is_seed(const PointItem* item) noexcept;
private:
	type_tag _Mytag{ type_tag::SEED };
	QString _Mylabel;
};