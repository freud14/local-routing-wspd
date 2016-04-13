#ifndef POINT_NUMBERS_GRAPHICS_ITEM_H
#define POINT_NUMBERS_GRAPHICS_ITEM_H

#include <CGAL/Bbox_2.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Qt/PainterOstream.h>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/Converter.h>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>


template <typename P>
class PointNumbersGraphicsItem : public CGAL::Qt::GraphicsItem
{
  typedef typename std::iterator_traits<typename P::iterator>::value_type Point_2;
  typedef typename CGAL::Kernel_traits<Point_2>::Kernel Traits;
  typedef typename Traits::Segment_2 Segment_2;

public:
  PointNumbersGraphicsItem(P* p_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_);

  void modelChanged();

public:
  QRectF boundingRect() const;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);


  const QPen& verticesPen() const
  {
    return vertices_pen;
  }

  void setVerticesPen(const QPen& pen)
  {
    vertices_pen = pen;
  }

  bool drawVertices() const
  {
    return draw_vertices;
  }

  void setDrawVertices(const bool b)
  {
    draw_vertices = b;
    update();
  }

protected:
  void updateBoundingBox();

  P * points;
  std::vector<int>* path;
  std::vector<int>* t_path;
  std::vector<Segment_2>* edges;
  QPainter* m_painter;
  CGAL::Qt::PainterOstream<Traits> painterostream;


  QRectF bounding_rect;

  QPen vertices_pen;
  bool draw_vertices;
};


template <typename P>
PointNumbersGraphicsItem<P>::PointNumbersGraphicsItem(P * p_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_)
  :  points(p_), path(path_), t_path(t_path_), edges(edges_), painterostream(0),  draw_vertices(true)
{
  setVerticesPen(QPen(::Qt::red, 10.));
  if(points->size() == 0){
    this->hide();
  }
  updateBoundingBox();
  setZValue(2);
}

template <typename P>
QRectF
PointNumbersGraphicsItem<P>::boundingRect() const
{
  return bounding_rect;
}




template <typename P>
void
PointNumbersGraphicsItem<P>::paint(QPainter *painter,
                                    const QStyleOptionGraphicsItem * /*option*/,
                                    QWidget * /*widget*/)
{
  if(drawVertices()) {
    CGAL::Qt::Converter<Traits> convert;

    CGAL::Qt::PainterOstream<Traits> painterostream = CGAL::Qt::PainterOstream<Traits>(painter);
    painter->setPen(QPen(Qt::red, 0));
    for (int i = 0; i < edges->size(); i++) {
      painterostream << edges->at(i);
    }
    painter->setPen(QPen(Qt::green, 1));
    for (int i = 1; i < t_path->size(); i++) {
      painterostream << Segment_2(points->at(t_path->at(i-1)), points->at(t_path->at(i)));
    }
    painter->setPen(QPen(Qt::blue, 1));
    for (int i = 1; i < path->size(); i++) {
      painterostream << Segment_2(points->at(path->at(i-1)), points->at(path->at(i)));
    }
    painter->setPen(verticesPen());

    QFont font;
    font.setPointSize(15);
    font.setBold(true);
    painter->setFont(font);
    QMatrix matrix = painter->matrix();
    painter->resetMatrix();
    int i = 0;
    for(typename P::iterator it = points->begin();
        it != points->end();
        it++) {
      QPointF point = matrix.map(convert(*it));
      painter->drawText(QRectF(point, QSizeF(50., 50.)), Qt::AlignLeft, QString::number(i));
      i++;
    }
  }
}

// We let the bounding box only grow, so that when vertices get removed
// the maximal bbox gets refreshed in the GraphicsView
template <typename P>
void
PointNumbersGraphicsItem<P>::updateBoundingBox()
{
  CGAL::Qt::Converter<Traits> convert;
  prepareGeometryChange();
  if(points->size() == 0){
    return;
  }
  bounding_rect = convert(CGAL::bounding_box(points->begin(), points->end()));
}


template <typename P>
void
PointNumbersGraphicsItem<P>::modelChanged()
{
  if((points->size() == 0) ){
    this->hide();
  } else if((points->size() > 0) && (! this->isVisible())){
    this->show();
  }
  updateBoundingBox();
  update();
}

#endif // POINT_NUMBERS_GRAPHICS_ITEM_H
