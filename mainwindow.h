#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <sstream>

#include "ui_mainwindow.h"
#include <CGAL/Qt/DemosMainWindow.h>
#include <CGAL/Object.h>
#include <CGAL/Qt/Converter.h>
#include "PointNumbersGraphicsItem.h"
#include "WSPDGraphicsItem.h"
#include <CGAL/Qt/GraphicsViewCircleInput.h>
#include "GraphicsViewPointInput.h"

#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_adapter.h>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/tuple/tuple.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Split_tree.h>
#include <CGAL/WSPD.h>
#include <CGAL/Split_tree_traits_2.h>
#include "Path_wspd.h"

enum {
  EMPTY,
  BIGGER_SMALLEST_BBOX,
  BIGGEST_BBOX_WS,
  BIGGEST_BBOX,
  DIRECTION,
  MONOTONE_X,
  EUCLIDEAN,
  SRC_INSIDE,
  EDGE_INSIDE
};

class MainWindow :
  public CGAL::Qt::DemosMainWindow,
  public Ui::MainWindow
{
  Q_OBJECT
private:
  typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
  typedef CGAL::Split_tree_traits_2<K>                            Traits;
  typedef CGAL::Split_tree<Traits>                                Split_tree;
  typedef Path_wspd<Traits>                                       Path_wspd_type;
  typedef typename Path_wspd_type::Well_separated_pair            Well_separated_pair;
  typedef typename Path_wspd_type::Well_separated_pair_iterator   Well_separated_pair_iterator;
  typedef typename Path_wspd_type::Node_const_handle              Node_const_handle;

  typedef K::Point_2                                              Point_2;
  typedef K::Circle_2                                             Circle_2;
  typedef K::Iso_rectangle_2                                      Iso_rectangle_2;
  typedef K::Segment_2                                            Segment_2;
  typedef K::Vector_2                                             Vector_2;
  typedef std::vector<Point_2>                                    Point_vector;

  typedef boost::tuple<Point_2,int> Point_and_int;
  typedef CGAL::Search_traits_adapter<Point_and_int,
    CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
    Traits>                                                      KdTreeTraits;
  typedef CGAL::Kd_tree<KdTreeTraits>                            Point_tree;
  typedef CGAL::Fuzzy_iso_box<KdTreeTraits>                      Fuzzy_iso_box;
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
  void saveAs(QString filename, bool savePath = false);

  void on_actionClear_triggered();

  void on_actionRecenter_triggered();

  virtual void open(QString fileName);

  void swapSourceDest();

  void findPath();

  void erasePath();

  void displayWsp();

  void eraseWsp();

  void randomTests();

  void displayCandidates();

  void eraseCandidates();

  void displayBboxes();

  void eraseBboxes();
Q_SIGNALS:
  void changed();

private:
  void setPathField();

  void resetWspd();

  void setupWspd();

  void addOptions(QComboBox* list, bool addEmpty = true, int default_option = 0);

  void setParameters(QComboBox* list, int index, Path_parameters& params);

  Path_parameters getParameters();
private:
  QGraphicsScene scene;
  CGAL::Qt::Converter<K> convert;

  WSPDGraphicsItem<Traits>* wspd_item;
  PointNumbersGraphicsItem<K,Traits>* pngi;

  CGAL::Qt::GraphicsViewPointInput<K>* gvpi;

  std::ostringstream out;

  Path_wspd_type wspd;
  double s;
  Point_vector points;
  std::vector<int> path_found;
  std::vector<int> t_path_found;
  std::vector<Segment_2> edges;
  std::vector<Iso_rectangle_2> bboxes;
  std::vector<std::pair<Circle_2, Circle_2> > pairs;
  std::pair<Circle_2, Circle_2>* wsp_pair;

  Point_tree* points_tree;
};

#endif // MAINWINDOW_H
