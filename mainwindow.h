#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <sstream>

#include "ui_mainwindow.h"
#include <CGAL/Qt/DemosMainWindow.h>
#include <CGAL/Object.h>
#include <CGAL/Qt/Converter.h>
#include <CGAL/Qt/PointsGraphicsItem.h>
#include "PointNumbersGraphicsItem.h"
#include "WSPDGraphicsItem.h"
#include <CGAL/Qt/GraphicsViewCircleInput.h>
#include "GraphicsViewPointInput.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <CGAL/Split_tree_traits_2.h>
#include "Path_wspd.h"

class MainWindow :
  public CGAL::Qt::DemosMainWindow,
  public Ui::MainWindow
{
  Q_OBJECT
private:
  typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
  typedef CGAL::Split_tree_traits_2<K>                            Traits;
  typedef CGAL::Split_tree<Traits>                                Split_tree;
  //typedef CGAL::WSPD<Traits>                                      WSPD;
  //typedef typename WSPD::Well_separated_pair                      Well_separated_pair;
  typedef Path_wspd<Traits>                                       Path_wspd_type;
  typedef typename Path_wspd_type::Well_separated_pair            Well_separated_pair;
  typedef typename Path_wspd_type::Node                           Node;

  typedef K::Point_2                                              Point_2;
  typedef K::Circle_2                                             Circle_2;
  typedef K::Iso_rectangle_2                                      Iso_rectangle_2;
  typedef std::vector<Point_2>                                    Point_vector;

public:
  MainWindow();
  ~MainWindow();

public Q_SLOTS:

  void processInput(CGAL::Object o);

  void on_actionInsertPoint_toggled(bool checked);

  void on_actionShowWspd_toggled(bool checked);

  void on_actionShowBoundingBoxes_toggled(bool checked);

  void on_actionInsertRandomPoints_triggered();

  void on_actionSetSeparationRatio_triggered();

  void on_actionLoadPoints_triggered();

  void on_actionSavePoints_triggered();

  void on_actionClear_triggered();

  void on_actionRecenter_triggered();

  virtual void open(QString fileName);

  void swapSourceDest();

  void findPath();

  void erasePath();

  void randomTests();
Q_SIGNALS:
  void changed();

private:
  void set_path_field();

  void reset_wspd();

private:
  QGraphicsScene scene;
  CGAL::Qt::Converter<K> convert;

  CGAL::Qt::PointsGraphicsItem<Point_vector>* pgi;
  WSPDGraphicsItem<Traits>* wspd_item;
  PointNumbersGraphicsItem<Point_vector>* pngi;

  CGAL::Qt::GraphicsViewPointInput<K>* gvpi;

  std::ostringstream out;

  //WSPD wspd;
  Path_wspd_type wspd;
  double s;
  Point_vector points;
  std::vector<int> path_found;
};

#endif // MAINWINDOW_H
