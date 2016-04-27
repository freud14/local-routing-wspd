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
  typedef typename Traits::FT FT;
  typedef typename Traits::Segment_2 Segment_2;
  typedef typename Traits::Iso_rectangle_2 Iso_rectangle_2;
  typedef typename Traits::Circle_2 Circle_2;
public:
  PointNumbersGraphicsItem(P* p_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_, std::vector<Iso_rectangle_2>* bboxes_, std::vector<std::pair<Circle_2, Circle_2> >* pairs_);

  void modelChanged();

public:
  QRectF boundingRect() const;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  void draw(QPainter *painter, QMatrix& matrix, const Segment_2& segment);
  void draw(QPainter *painter, QMatrix& matrix, const Iso_rectangle_2& rect);
  void draw(QPainter *painter, QMatrix& matrix, const Circle_2& circle);

  Segment_2 segment_between_circles(const Circle_2& c1, const Circle_2& c2) const;

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
  std::vector<Iso_rectangle_2>* bboxes;
  std::vector<std::pair<Circle_2, Circle_2> >* pairs;
  QPainter* m_painter;
  CGAL::Qt::Converter<Traits> convert;


  QRectF bounding_rect;

  QPen vertices_pen;
  bool draw_vertices;
};


template <typename P>
PointNumbersGraphicsItem<P>::PointNumbersGraphicsItem(P * p_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_, std::vector<Iso_rectangle_2>* bboxes_, std::vector<std::pair<Circle_2, Circle_2> >* pairs_)
  :  points(p_), path(path_), t_path(t_path_), edges(edges_), bboxes(bboxes_), pairs(pairs_), draw_vertices(true)
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
    QMatrix matrix = painter->matrix();
    painter->resetMatrix();

    painter->setPen(QPen(Qt::green, 2));
    for (int i = 1; i < t_path->size(); i++) {
      draw(painter, matrix, Segment_2(points->at(t_path->at(i-1)), points->at(t_path->at(i))));
    }
    painter->setPen(QPen(Qt::blue, 2));
    for (int i = 1; i < path->size(); i++) {
      draw(painter, matrix, Segment_2(points->at(path->at(i-1)), points->at(path->at(i))));
    }
    if(edges->size() > 0) {
      painter->setPen(QPen(Qt::cyan, 2));
      draw(painter, matrix, edges->at(edges->size() - 1));
      painter->setPen(QPen(Qt::red, 0));
      for (int i = 0; i < edges->size() - 1; i++) {
        draw(painter, matrix, edges->at(i));
      }
    }
    painter->setPen(QPen(Qt::red, 3));
    for (int i = 0; i < bboxes->size(); i++) {
      draw(painter, matrix, bboxes->at(i));
    }

    painter->setPen(QPen(Qt::black, 0));
    for(typename std::vector<std::pair<Circle_2, Circle_2> >::iterator it = pairs->begin(); it < pairs->end(); it++) {
      std::pair<Circle_2, Circle_2> pair = *it;
      if(pair.first.squared_radius() > 0) {
        draw(painter, matrix, pair.first);
      }
      if(pair.second.squared_radius() > 0) {
        draw(painter, matrix, pair.second);
      }
      draw(painter, matrix, segment_between_circles(pair.first, pair.second));
    }

    painter->setPen(verticesPen());
    QFont font;
    font.setPointSize(15);
    font.setBold(true);
    painter->setFont(font);
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

template <typename P>
void
PointNumbersGraphicsItem<P>::draw(QPainter *painter, QMatrix& matrix, const Segment_2& segment) {
  painter->drawLine(matrix.map(convert(segment)));
}

template <typename P>
void
PointNumbersGraphicsItem<P>::draw(QPainter *painter, QMatrix& matrix, const Iso_rectangle_2& rect) {
  painter->drawRect(matrix.map(convert(rect)).boundingRect());
}

template <typename P>
void
PointNumbersGraphicsItem<P>::draw(QPainter *painter, QMatrix& matrix, const Circle_2& circle) {
  painter->drawEllipse(matrix.map(convert(circle.bbox())).boundingRect());
}

template <typename P>
typename PointNumbersGraphicsItem<P>::Segment_2 PointNumbersGraphicsItem<P>::segment_between_circles(const Circle_2& c1, const Circle_2& c2) const {
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
