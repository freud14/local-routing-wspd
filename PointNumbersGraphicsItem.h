#ifndef POINT_NUMBERS_GRAPHICS_ITEM_H
#define POINT_NUMBERS_GRAPHICS_ITEM_H
#include <algorithm>

#include <CGAL/bounding_box.h>
#include <CGAL/Qt/PainterOstream.h>
#include <CGAL/Qt/GraphicsItem.h>
#include <CGAL/Qt/Converter.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Fuzzy_iso_box.h>
#include <CGAL/Search_traits_adapter.h>
#include <boost/tuple/tuple.hpp>

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>
#include <QFontMetrics>


template <class K, class Traits>
class PointNumbersGraphicsItem : public CGAL::Qt::GraphicsItem
{
  typedef typename K::Point_2 Point_2;
  typedef typename K::FT FT;
  typedef typename K::Segment_2 Segment_2;
  typedef typename K::Iso_rectangle_2 Iso_rectangle_2;
  typedef typename K::Circle_2 Circle_2;
  typedef boost::tuple<Point_2,int> Point_and_int;
  typedef CGAL::Search_traits_adapter<Point_and_int,
    CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
    Traits>                                                      KdTreeTraits;
  typedef CGAL::Kd_tree<KdTreeTraits>                            Tree;
  typedef CGAL::Fuzzy_iso_box<KdTreeTraits>                      Fuzzy_iso_box;
public:
  PointNumbersGraphicsItem(QGraphicsView* view, Tree*& tree_, std::vector<Point_2>* points_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_, std::vector<Iso_rectangle_2>* bboxes_, std::vector<std::pair<Circle_2, Circle_2> >* pairs_, std::pair<Circle_2, Circle_2>*& wsp_pair_);

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

  Tree*& tree;
  std::vector<Point_2>* points;
  std::vector<int>* path;
  std::vector<int>* t_path;
  std::vector<Segment_2>* edges;
  std::vector<Iso_rectangle_2>* bboxes;
  std::vector<std::pair<Circle_2, Circle_2> >* pairs;
  std::pair<Circle_2, Circle_2>*& wsp_pair;
  QPainter* m_painter;
  CGAL::Qt::Converter<K> convert;

  QGraphicsView* graphicsView;


  QRectF bounding_rect;

  QPen vertices_pen;
  bool draw_vertices;
};


template <class K, class Traits>
PointNumbersGraphicsItem<K,Traits>::PointNumbersGraphicsItem(QGraphicsView* view, Tree*& tree_, std::vector<Point_2>* points_, std::vector<int>* path_, std::vector<int>* t_path_, std::vector<Segment_2>* edges_, std::vector<Iso_rectangle_2>* bboxes_, std::vector<std::pair<Circle_2, Circle_2> >* pairs_, std::pair<Circle_2, Circle_2>*& wsp_pair_)
  :  graphicsView(view), tree(tree_), points(points_), path(path_), t_path(t_path_), edges(edges_), bboxes(bboxes_), pairs(pairs_), wsp_pair(wsp_pair_), draw_vertices(true)
{
  setVerticesPen(QPen(::Qt::red, 10.));
  if(points->size() == 0){
    this->hide();
  }
  updateBoundingBox();
  setZValue(2);
  setFlag(QGraphicsItem::ItemUsesExtendedStyleOption, true);
}

template <class K, class Traits>
QRectF
PointNumbersGraphicsItem<K,Traits>::boundingRect() const
{
  return bounding_rect;
}




template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::paint(QPainter *painter,
                                    const QStyleOptionGraphicsItem * option,
                                    QWidget * /*widget*/)
{
  if(drawVertices()) {
    QMatrix matrix = painter->matrix();
    painter->resetMatrix();

    bounding_rect = graphicsView->mapToScene(graphicsView->rect()).boundingRect();

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
      painter->setPen(QPen(Qt::red, 1));
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

    painter->setPen(QPen(Qt::black, 0));
    if(wsp_pair) {
      if(wsp_pair->first.squared_radius() > 0) {
        draw(painter, matrix, wsp_pair->first);
      }
      if(wsp_pair->second.squared_radius() > 0) {
        draw(painter, matrix, wsp_pair->second);
      }
      draw(painter, matrix, segment_between_circles(wsp_pair->first, wsp_pair->second));
    }

    int count = 0;
    QFont font;
    font.setPointSize(15);
    font.setBold(true);
    painter->setFont(font);
    QFontMetrics fontMetric(font);
    std::vector<Point_and_int> points_to_display;
    Iso_rectangle_2 window = convert(bounding_rect);
    Fuzzy_iso_box fib(window.min(), window.max());
    tree->search(std::back_inserter(points_to_display), fib);
    std::random_shuffle(points_to_display.begin(), points_to_display.end());
    for(typename std::vector<Point_and_int>::iterator it = points_to_display.begin(); it != points_to_display.end() && count < 100; it++) {
      QPointF point = matrix.map(convert(get<0>(*it)));
      painter->setPen(verticesPen());

      QString number = QString::number(get<1>(*it));
      QRectF textRect(point, fontMetric.size(0, number));
      painter->drawText(textRect, Qt::AlignLeft, number);

      painter->setPen(QPen(Qt::black, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      painter->drawPoint(point);

      count++;
    }
  }
}

template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::draw(QPainter *painter, QMatrix& matrix, const Segment_2& segment) {
  painter->drawLine(matrix.map(convert(segment)));
}

template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::draw(QPainter *painter, QMatrix& matrix, const Iso_rectangle_2& rect) {
  painter->drawRect(matrix.map(convert(rect)).boundingRect());
}

template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::draw(QPainter *painter, QMatrix& matrix, const Circle_2& circle) {
  painter->drawEllipse(matrix.map(convert(circle.bbox())).boundingRect());
}

template <class K, class Traits>
typename PointNumbersGraphicsItem<K,Traits>::Segment_2 PointNumbersGraphicsItem<K,Traits>::segment_between_circles(const Circle_2& c1, const Circle_2& c2) const {
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
template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::updateBoundingBox()
{
  prepareGeometryChange();
  bounding_rect = graphicsView->mapToScene(graphicsView->rect()).boundingRect();
  /*if(points->size() == 0){
    return;
  }
  bounding_rect = convert(CGAL::bounding_box(points->begin(), points->end()));*/
}


template <class K, class Traits>
void
PointNumbersGraphicsItem<K,Traits>::modelChanged()
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
