#ifndef WSPD_GRAPHICS_ITEM_H
#define WSPD_GRAPHICS_ITEM_H

#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <CGAL/Bbox_2.h>

#include <CGAL/Qt/PainterOstream.h>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/Converter.h>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

template <typename Traits>
class WSPDGraphicsItem : public CGAL::Qt::GraphicsItem
{
  typedef typename Traits::K                                      K;
  typedef CGAL::Split_tree<Traits>                                Split_tree;
  typedef typename Split_tree::Bounding_box_iterator              Bounding_box_iterator;
  typedef CGAL::WSPD<Traits>                                      WSPD;
  typedef typename WSPD::Well_separated_pair_iterator             Well_separated_pair_iterator;
  typedef typename WSPD::Well_separated_pair                      Well_separated_pair;

  typedef typename K::FT                                          FT;
  typedef typename K::Point_2                                     Point_2;
  typedef typename K::Segment_2                                   Segment_2;
  typedef typename K::Circle_2                                    Circle_2;
  typedef typename K::Iso_rectangle_2                             Iso_rectangle_2;
  typedef CGAL::Bbox_2                                            Bbox_2;
public:
  WSPDGraphicsItem(WSPD* wspd_);

  void modelChanged();

public:
  QRectF boundingRect() const;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

  Segment_2 segment_between_circles(const Circle_2& c1, const Circle_2& c2) const;

  const QPen& verticesPen() const
  {
    return vertices_pen;
  }

  const QPen& edgesPen() const
  {
    return edges_pen;
  }

  void setVerticesPen(const QPen& pen)
  {
    vertices_pen = pen;
  }

  void setEdgesPen(const QPen& pen)
  {
    edges_pen = pen;
  }

  bool drawWspd() const
  {
    return draw_wspd;
  }

  void setDrawWspd(const bool b)
  {
    draw_wspd = b;
    update();
  }

  bool drawBoundingBoxes() const
  {
    return draw_bounding_boxes;
  }

  void setDrawBoundingBoxes(const bool b)
  {
    draw_bounding_boxes = b;
    update();
  }

protected:
  void updateBoundingBox();

  WSPD * wspd;
  QPainter* m_painter;
  CGAL::Qt::PainterOstream<K> painterostream;

  QRectF bounding_rect;

  QPen vertices_pen;
  QPen edges_pen;
  bool draw_bounding_boxes;
  bool draw_wspd;
};


template <typename K>
WSPDGraphicsItem<K>::WSPDGraphicsItem(WSPD* wspd_)
  :  wspd(wspd_), painterostream(0),
     draw_bounding_boxes(true), draw_wspd(true)
{
  setVerticesPen(QPen(::Qt::red, 3.));
  setEdgesPen(QPen(::Qt::black, 0));
  updateBoundingBox();
  setZValue(3);
}

template <typename K>
QRectF WSPDGraphicsItem<K>::boundingRect() const
{
  return bounding_rect;
}

template <typename K>
void WSPDGraphicsItem<K>::paint(QPainter *painter,
                                    const QStyleOptionGraphicsItem * /*option*/,
                                    QWidget * /*widget*/)
{
  painter->setPen(this->edgesPen());
  painterostream = CGAL::Qt::PainterOstream<K>(painter);

  if(draw_wspd) {
    for(Well_separated_pair_iterator it = wspd->wspd_begin(); it < wspd->wspd_end(); it++) {
      const Well_separated_pair &pair = *it;
      Circle_2 c1 = pair.first->enclosing_circle();
      Circle_2 c2 = pair.second->enclosing_circle();
      if(!pair.first->is_leaf()) {
        painterostream << c1;
      }
      if(!pair.second->is_leaf()) {
        painterostream << c2;
      }
      painterostream << segment_between_circles(c1, c2);
    }
  }

  if(draw_bounding_boxes) {
    const Split_tree& tree = wspd->split_tree();
    for(Bounding_box_iterator it = tree.bounding_box_begin(); it < tree.bounding_box_end(); it++) {
      painterostream << *it;
    }
  }
}

template <typename K>
typename WSPDGraphicsItem<K>::Segment_2 WSPDGraphicsItem<K>::segment_between_circles(const Circle_2& c1, const Circle_2& c2) const {
  Point_2 p1 = c1.center();
  Point_2 p2 = c2.center();
  FT dx = p1.x() - p2.x();
  FT dy = p1.y() - p2.y();
  FT length = CGAL::sqrt(CGAL::squared_distance(p1, p2));
  FT r1 = CGAL::sqrt(c1.squared_radius());
  FT r2 = CGAL::sqrt(c2.squared_radius());
  Point_2 s1 = Point_2(p1.x() - r1/length*dx, p1.y() - r1/length*dy);
  Point_2 s2 = Point_2(p1.x() - (1-r2/length)*dx, p1.y() - (1-r2/length)*dy);
  return Segment_2(s1, s2);
}

template <typename K>
void WSPDGraphicsItem<K>::updateBoundingBox()
{
  Bbox_2 bbox;
  for(Well_separated_pair_iterator it = wspd->wspd_begin(); it < wspd->wspd_end(); it++) {
    const Well_separated_pair &pair = *it;
    Circle_2 c1 = pair.first->enclosing_circle();
    Circle_2 c2 = pair.second->enclosing_circle();
    bbox += c1.bbox() + c2.bbox();
  }
  CGAL::Qt::Converter<K> convert;
  bounding_rect = convert(bbox + convert(bounding_rect).bbox());
}


template <typename K>
void WSPDGraphicsItem<K>::modelChanged()
{
  updateBoundingBox();
  update();
}

#endif // WSPD_GRAPHICS_ITEM_H
