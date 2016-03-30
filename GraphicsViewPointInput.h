
#ifndef CGAL_QT_GRAPHICS_VIEW_POINT_INPUT
#define CGAL_QT_GRAPHICS_VIEW_POINT_INPUT

#include <CGAL/Qt/GraphicsViewInput.h>
#include <CGAL/Qt/Converter.h>
#include <QGraphicsSceneMouseEvent>
#include <QEvent>

namespace CGAL {
namespace Qt {

template <typename K>
class GraphicsViewPointInput : public GraphicsViewInput
{
private:
  typedef typename K::Point_2 Point_2;

public:
  GraphicsViewPointInput(QObject* parent, QGraphicsScene* s);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent *event);
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
  bool eventFilter(QObject *obj, QEvent *event);

private:
  Converter<K> convert;
  QGraphicsScene *scene_;
  Point_2 p;
};


template <typename K>
GraphicsViewPointInput<K>::GraphicsViewPointInput(QObject* parent, QGraphicsScene* s)
  :  GraphicsViewInput(parent), scene_(s)
{}

template <typename K>
void
GraphicsViewPointInput<K>::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
  p = convert(event->scenePos());
}

template <typename K>
void
GraphicsViewPointInput<K>::mouseReleaseEvent(QGraphicsSceneMouseEvent * /*event*/)
{
  Q_EMIT( generate(CGAL::make_object(p)));
}

template <typename K>
bool
GraphicsViewPointInput<K>::eventFilter(QObject *obj, QEvent *event)
{
  if (event->type() == QEvent::GraphicsSceneMousePress) {
    QGraphicsSceneMouseEvent *mouseEvent = static_cast<QGraphicsSceneMouseEvent *>(event);
    mousePressEvent(mouseEvent);
    return true;
  } else if (event->type() == QEvent::GraphicsSceneMouseRelease) {
    QGraphicsSceneMouseEvent *mouseEvent = static_cast<QGraphicsSceneMouseEvent *>(event);
    mouseReleaseEvent(mouseEvent);
    return true;
  } else{
    // standard event processing
    return QObject::eventFilter(obj, event);
  }
}


} // namespace Qt
} // namespace CGAL

#endif // CGAL_QT_GRAPHICS_VIEW_POINT_INPUT
