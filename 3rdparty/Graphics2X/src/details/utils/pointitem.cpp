#include <QtGui>
#include "utils/pointitem.h"

SeedItem::SeedItem()
	: QGraphicsItem()
{
	QPainterPath circle;
	circle.addEllipse(-8,-22,16,16);
	
	QPainterPath inner_circle;
	circle.addEllipse({-8.+8, -22.+8}, 4, 4);

	QPainterPath triangle;
	triangle.moveTo(-6,-10);
	triangle.lineTo(6,-10);
	triangle.lineTo(0,0);
	triangle.lineTo(-6,-10);

	_Mypath = triangle + circle;
	_Mypath -= inner_circle;

	setFlag(QGraphicsItem::ItemIsMovable, true);
	setFlag(QGraphicsItem::ItemIgnoresTransformations,false);
	setZValue(2.0);
}

SeedItem::~SeedItem() {
}

QRectF SeedItem::boundingRect() const
{
	return QRectF(-8,-24,16,24);
}

void SeedItem::paint( QPainter *painter, 
	const QStyleOptionGraphicsItem *option, 
	QWidget *widget )
{
	painter->setRenderHint(QPainter::Antialiasing, true);
	painter->setPen(Qt::NoPen);
	painter->setBrush(QColor(0,255,0,255)); //here to change the color and transparency of the marker.
	painter->drawPath(_Mypath);
}

QPainterPath SeedItem::shape() const {
	return _Mypath;
}

QPainterPath* SeedItem::mutable_shape() const noexcept
{
	return std::addressof(_Mypath);
}

PointItem::PointItem(type_tag type) noexcept
	:SeedItem(), _Mytag(type)
{
	QPainterPath path;
	switch (_Mytag)
	{
	case PointItem::type_tag::CROSS: {
		path.addRect({ -1, -8, 2, 16 });
		path.addRect({ -8, -1, 16, 2 });
	} break;
	case PointItem::type_tag::FCROSS: {
		path.addRect(0-16, 0-16, 8, 2);   //< Left-top
		path.addRect(0-16, 0-16, 2, 8);
		path.addRect(24-16, 0-16, 8, 2);  //< Right-top
		path.addRect(30-16, 0-16, 2, 8);
		path.addRect(0-16, 24-16, 2, 8);  //< Left-bottom
		path.addRect(0-16, 30-16, 8, 2);
		path.addRect(24-16, 30-16, 8, 2); //< Left-bottom
		path.addRect(30-16, 24-16, 2, 8);
		path.addRect({ -1, -8, 2, 16 });  //< Inner cross
		path.addRect({ -8, -1, 16, 2 });
	} break;
	case PointItem::type_tag::FDOT: {
		path.addEllipse(-8, -24, 16, 16);
	} break;
	case PointItem::type_tag::HDOT: {
		path.addEllipse(-8, -24, 16, 16);
		decltype(path) hole;
		hole.addEllipse({ -8. + 8, -24. + 8 }, 4, 4);
		path -= hole;
	} break;
	default: break;
	}
	if(_Mytag!=type_tag::SEED) *mutable_shape() = path;
}

void PointItem::set_label(QString label) noexcept
{
	_Mylabel = label;
	auto font = QFont("Arial");
	font.setPointSize(8);
	const auto x = _Mytag == type_tag::SEED ? 8 : 3;
	const auto y = _Mytag == type_tag::SEED ? -20 : -3;
	mutable_shape()->addText(x, y, font, _Mylabel);
}

void PointItem::set_label(size_t id)  noexcept
{
	set_label(QString::number(id));
}

size_t PointItem::label() const noexcept
{
	auto ok = false;
	const auto value = _Mylabel.toInt(&ok);
	return ok ? value : std::numeric_limits<size_t>::quiet_NaN();
}

bool is_seed(const PointItem* item) noexcept
{
	return item->_Mytag == PointItem::type_t::SEED;
}
