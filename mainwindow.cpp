#include <fstream>

// CGAL headers
#include <CGAL/point_generators_2.h>

// Qt headers
#include <QtGui>
#include <QString>
#include <QActionGroup>
#include <QFileDialog>
#include <QInputDialog>
#include <QScrollBar>
 #include <QComboBox>

// for viewportsBbox
#include <CGAL/Qt/utility.h>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow() :
    DemosMainWindow(), s(2.0), wspd(2, 2.0)
{
  setupUi(this);

  setPathField();
  QObject::connect(findPathButton, SIGNAL (released()),this, SLOT (findPath()));
  QObject::connect(erasePathButton, SIGNAL (released()),this, SLOT (erasePath()));
  QObject::connect(swapButton, SIGNAL (released()),this, SLOT (swapSourceDest()));
  QObject::connect(randomTestsButton, SIGNAL (released()),this, SLOT (randomTests()));
  QObject::connect(displayCandidatesButton, SIGNAL (released()),this, SLOT (displayCandidates()));
  QObject::connect(eraseCandidatesButton, SIGNAL (released()),this, SLOT (eraseCandidates()));
  QObject::connect(displayBboxesButton, SIGNAL (released()),this, SLOT (displayBboxes()));
  QObject::connect(eraseBboxesButton, SIGNAL (released()),this, SLOT (eraseBboxes()));

  pgi = new CGAL::Qt::PointsGraphicsItem<Point_vector>(&points);
  QObject::connect(this, SIGNAL(changed()), pgi, SLOT(modelChanged()));
  pgi->setVerticesPen(QPen(Qt::black, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  scene.addItem(pgi);

  pngi = new PointNumbersGraphicsItem<Point_vector>(&points, &path_found, &t_path_found, &edges, &bboxes, &pairs);
  QObject::connect(this, SIGNAL(changed()), pngi, SLOT(modelChanged()));
  scene.addItem(pngi);

  wspd_item = new WSPDGraphicsItem<Traits>(&wspd);
  QObject::connect(this, SIGNAL(changed()), wspd_item, SLOT(modelChanged()));
  wspd_item->setVerticesPen(QPen(Qt::black, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  scene.addItem(wspd_item);

  gvpi = new CGAL::Qt::GraphicsViewPointInput<K>(this, &scene);
  QObject::connect(gvpi, SIGNAL(generate(CGAL::Object)),
  		   this, SLOT(processInput(CGAL::Object)));
  //
  // Manual handling of actions
  //

  QObject::connect(this->actionQuit, SIGNAL(triggered()),
       this, SLOT(close()));

  // Check two actions
  //this->actionInsertPoint->setChecked(true);
  this->actionShowWspd->setChecked(true);
  wspd_item->setDrawWspd(true);
  this->actionShowBoundingBoxes->setChecked(false);
  wspd_item->setDrawBoundingBoxes(false);

  numberPointsTests->setMaximum(1000000);
  numberPointsTests->setValue(100);

  //
  // Setup the scene and the view
  //
  scene.setItemIndexMethod(QGraphicsScene::NoIndex);
  scene.setSceneRect(-100, -100, 100, 100);
  this->graphicsView->setScene(&scene);
  this->graphicsView->setMouseTracking(true);

  // Turn the vertical axis upside down
  this->graphicsView->matrix().scale(1, -1);

  // The navigation adds zooming and translation functionality to the
  // QGraphicsView
  this->addNavigation(this->graphicsView);

  this->setupStatusBar();
  this->setupOptionsMenu();
  //this->addAboutDemo(":/cgal/help/about_Delaunay_triangulation_2.html");
  this->addAboutCGAL();
  this->setupExportSVG(actionExport_SVG, graphicsView);

  this->addRecentFiles(this->menuFile, this->actionQuit);
  connect(this, SIGNAL(openRecentFile(QString)),
  this, SLOT(open(QString)));

  addOptions(initFilterList, false);
  addOptions(secondFilterList);
  addOptions(thirdFilterList);
  addOptions(fourthFilterList);
  addOptions(fifthFilterList);
}

MainWindow::~MainWindow()
{
  delete pgi;
  delete pngi;
  delete wspd_item;
  delete gvpi;
}

void MainWindow::processInput(CGAL::Object o)
{
  Point_2 p;
  if(CGAL::assign(p, o)){
    if(std::find(points.begin(), points.end(), p) == points.end()) {
      points.push_back(p);
      resetWspd();
      Q_EMIT( changed());
    }
  }
}

void MainWindow::on_actionInsertPoint_toggled(bool checked)
{
  if(checked){
    scene.installEventFilter(gvpi);
    scene.installEventFilter(pgi);
  } else {
    scene.removeEventFilter(gvpi);
    scene.removeEventFilter(pgi);
  }
}

void MainWindow::on_actionShowWspd_toggled(bool checked)
{
  wspd_item->setDrawWspd(checked);
}


void MainWindow::on_actionShowBoundingBoxes_toggled(bool checked)
{
  wspd_item->setDrawBoundingBoxes(checked);
}

void MainWindow::on_actionClear_triggered()
{
  points.clear();
  resetWspd();
  Q_EMIT( changed());
}


void MainWindow::on_actionInsertRandomPoints_triggered()
{
  QRectF rect = CGAL::Qt::viewportsBbox(&scene);
  Iso_rectangle_2 isor = convert(rect);
  CGAL::Random_points_in_iso_rectangle_2<Point_2> pg((isor.min)(), (isor.max)());

  const int number_of_points =
    QInputDialog::getInt(this,
                             tr("Number of random points"),
                             tr("Enter number of random points"), 10, 0);

  // wait cursor
  QApplication::setOverrideCursor(Qt::WaitCursor);
  points.reserve(points.size() + number_of_points);
  for(int i = 0; i < number_of_points; ++i){
    points.push_back(*pg++);
  }
  resetWspd();
  // default cursor
  QApplication::setOverrideCursor(Qt::ArrowCursor);
  Q_EMIT( changed());
}

void MainWindow::on_actionSetSeparationRatio_triggered() {
  s =
    QInputDialog::getDouble(this,
                             tr("Separation ratio"),
                             tr("Enter the separation ratio"), s, 0.0);
  path_found.clear();
  t_path_found.clear();
  wspd.separation_ratio(s);
  Q_EMIT( changed());
}

void MainWindow::on_actionLoadPoints_triggered()
{
  QString fileName = QFileDialog::getOpenFileName(this,
						  tr("Open Points file"),
						  ".");
  if(! fileName.isEmpty()){
    open(fileName);
  }
}

void MainWindow::open(QString fileName)
{
  // wait cursor
  QApplication::setOverrideCursor(Qt::WaitCursor);
  std::ifstream ifs(qPrintable(fileName));

  points.clear();

  std::string first_line;
  ifs >> first_line;
  QString q_first_line = first_line.c_str();
  int pos = 0;
  if(q_first_line.startsWith("s=")){
    s = q_first_line.split("=")[1].toDouble();
    wspd.separation_ratio(s);

    pos = ifs.tellg();
    ifs >> first_line;
    q_first_line = first_line.c_str();
  }

  int from = 0;
  int to = 0;
  if(q_first_line.startsWith("path=")){
    from = q_first_line.split("=")[1].split(",")[0].toInt();
    to = q_first_line.split("=")[1].split(",")[1].toInt();
  }
  else {
    ifs.seekg(pos);
  }

  Point_2 p;
  while(ifs >> p) {
    points.push_back(p);
  }
  resetWspd();

  textFrom->setValue(from);
  textTo->setValue(to);
  findPathButton->click();

  actionRecenter->trigger();

  // default cursor
  QApplication::restoreOverrideCursor();
  this->addToRecentFiles(fileName);
  actionRecenter->trigger();
  Q_EMIT(changed());
}

void MainWindow::on_actionSavePoints_triggered()
{
  QString fileName = QFileDialog::getSaveFileName(this,
						  tr("Save points"),
						  ".");
  if(! fileName.isEmpty()){
    std::ofstream ofs(qPrintable(fileName));
    for(Point_vector::iterator it = points.begin(); it!= points.end(); ++it)
    {
      ofs << *it << std::endl;
    }
  }
}

void MainWindow::on_actionRecenter_triggered()
{
  this->graphicsView->setSceneRect(wspd_item->boundingRect());
  this->graphicsView->fitInView(wspd_item->boundingRect(), Qt::KeepAspectRatio);
}

void MainWindow::swapSourceDest()
{
  int oldFrom = textFrom->value();
  int oldTo = textTo->value();
  textFrom->setValue(oldTo);
  textTo->setValue(oldFrom);
}

void MainWindow::findPath()
{
  Path_parameters params = getParameters();
  path_found = wspd.find_path(textFrom->value(), textTo->value(), params, detourTwoEdgesCheck->isChecked(), out, true);
  t_path_found = wspd.find_t_path(path_found);
  out << "t-path: " << t_path_found << std::endl;
  out << "----------------------------------------------" << std::endl;
  results->setText(out.str().c_str());
  results->verticalScrollBar()->triggerAction(QAbstractSlider::SliderToMaximum);
  erasePathButton->setEnabled(true);
  Q_EMIT( changed());
}

void MainWindow::erasePath()
{
  path_found.clear();
  t_path_found.clear();
  erasePathButton->setEnabled(false);
  Q_EMIT( changed());
}

void MainWindow::displayCandidates()
{
  std::vector<int> candidates;
  if(filtersCheck->isChecked()) {
    Path_parameters params = getParameters();
    candidates = wspd.get_candidates(params, textFrom->value(), textTo->value(), pointToDisplay->value());
  }
  else {
    candidates = wspd.get_neighbors(pointToDisplay->value());
  }
  Point_2 point = points[pointToDisplay->value()];
  edges.clear();
  for(int i = 0; i < candidates.size(); i++) {
    edges.push_back(Segment_2(point, points[candidates[i]]));
  }
  edges.push_back(Segment_2(point, points[textTo->value()]));
  eraseCandidatesButton->setEnabled(true);
  Q_EMIT( changed());
}

void MainWindow::eraseCandidates()
{
  edges.clear();
  eraseCandidatesButton->setEnabled(false);
  Q_EMIT( changed());
}

void MainWindow::displayBboxes()
{
  bboxes.clear();
  pairs.clear();
  if(pairsCheck->isChecked()) {
    std::vector<Well_separated_pair> rep = wspd.get_representative_pairs(bboxesToDisplay->value());
    for(Well_separated_pair_iterator it = rep.begin(); it < rep.end(); it++) {
      const Well_separated_pair &pair = *it;
      Circle_2 c1 = pair.a()->enclosing_circle();
      Circle_2 c2 = pair.b()->enclosing_circle();
      pairs.push_back(std::make_pair(c1, c2));
    }
  }
  bboxes = wspd.get_bboxes(bboxesToDisplay->value());
  eraseBboxesButton->setEnabled(true);
  Q_EMIT( changed());
}

void MainWindow::eraseBboxes()
{
  bboxes.clear();
  pairs.clear();
  eraseBboxesButton->setEnabled(false);
  Q_EMIT( changed());
}

void MainWindow::randomTests()
{
  //wspd.display_wspd(out);
  QRectF rect = CGAL::Qt::viewportsBbox(&scene);
  Iso_rectangle_2 isor = convert(rect);
  //CGAL::Random rand(42);
  CGAL::Random_points_in_iso_rectangle_2<Point_2> pg(isor.min(), isor.max());

  Path_parameters params = getParameters();
  int n = numberPointsTests->value();
  int nbTest = 1;
  points.clear();
  int from;
  int to;
  bool counter_example_found = false;
  for (int test = 0; test < nbTest; ++test) {
    points.reserve(n);
    for(int i = 0; i < n; ++i){
      points.push_back(*pg++);
    }
    std::vector<bool> is_tested;
    wspd.set(2, points.begin(), points.end());
    for (from = 0; from < points.size(); from++) {
      std::cout << from << std::endl;
      is_tested.assign(n, false);
      for (to = 0; to < points.size(); to++) {
        if(from != to && !is_tested[to]) {
          std::vector<int> path = wspd.find_path(from, to, params);
          if(path[path.size() - 1] != to || !wspd.verify_algo_induction_proof(path, detourTwoEdgesCheck->isChecked())) {
            path_found = wspd.find_path(from, to, params, detourTwoEdgesCheck->isChecked(), out, true);
            if(path_found[path_found.size() - 1] != to) {
              path_found.push_back(to);
            }
            t_path_found = wspd.find_t_path(path_found);
            out << "t-path: " << t_path_found << std::endl;

            counter_example_found = true;
            goto end_loops;
          }
          else {
            const Well_separated_pair& pair = wspd.get_wsp(from, to);
            Node_const_handle node = pair.a();
            if(pair.b()->bounding_box().bounded_side(points[to]) != -1) {
              node = pair.b();
            }
            std::vector<int> nodePoints = wspd.get_points(node);
            for(int i = 0; i < nodePoints.size(); i++) {
              is_tested[nodePoints[i]] = true;
            }
          }
        }
      }
    }
    points.clear();
  }
end_loops:
  if(counter_example_found) {
    edges.clear();
    bboxes.clear();
    pairs.clear();
    setPathField();
    textFrom->setValue(from);
    textTo->setValue(to);
  }
  else {
    resetWspd();
    out << "Test passed!" << std::endl;
  }
  Q_EMIT( changed());

  out << "----------------------------------------------" << std::endl;
  results->setText(out.str().c_str());
  results->verticalScrollBar()->triggerAction(QAbstractSlider::SliderToMaximum);
}

void MainWindow::resetWspd()
{
  path_found.clear();
  t_path_found.clear();
  edges.clear();
  bboxes.clear();
  pairs.clear();
  wspd.set(2, points.begin(), points.end());
  setPathField();
}

void MainWindow::setPathField()
{
  bool enabled = points.size() != 0;
  int max = points.size() - 1;
  if(!enabled) {
    max = 0;
  }
  textFrom->setMaximum(max);
  textFrom->setEnabled(enabled);
  textTo->setMaximum(max);
  textTo->setEnabled(enabled);
  swapButton->setEnabled(enabled);
  findPathButton->setEnabled(enabled);

  filtersCheck->setEnabled(enabled);
  pointToDisplay->setMaximum(max);
  pointToDisplay->setEnabled(enabled);
  displayCandidatesButton->setEnabled(enabled);

  pairsCheck->setEnabled(enabled);
  bboxesToDisplay->setMaximum(max);
  bboxesToDisplay->setEnabled(enabled);
  displayBboxesButton->setEnabled(enabled);

  erasePathButton->setEnabled(path_found.size() != 0);
  eraseCandidatesButton->setEnabled(edges.size() != 0);
  eraseBboxesButton->setEnabled(bboxes.size() != 0);
}

void MainWindow::addOptions(QComboBox* list, bool addEmpty) {
  if(addEmpty) {
    list->addItem("", QVariant(EMPTY));
  }
  list->addItem("Take bigger smallest", QVariant(BIGGER_SMALLEST_BBOX));
  list->addItem("Take biggest box well-separated", QVariant(BIGGEST_BBOX_WS));
  list->addItem("Take biggest box", QVariant(BIGGEST_BBOX));
  list->addItem("Direction", QVariant(DIRECTION));
  list->addItem("X-monotone", QVariant(MONOTONE_X));
  list->addItem("Euclidean", QVariant(EUCLIDEAN));
  //list->addItem("Source inside", QVariant(SRC_INSIDE));
  list->addItem("Take edge inside", QVariant(EDGE_INSIDE));
}

void MainWindow::setParameters(QComboBox* list, int index, Path_parameters& params) {
  if(list->currentData().isValid()) {
    switch(list->currentData().toInt()) {
      case EMPTY:
        break;
      case BIGGER_SMALLEST_BBOX:
        params.bigger_smallest_bbox = index;
        break;
      case BIGGEST_BBOX_WS:
        params.biggest_box_ws = index;
        break;
      case BIGGEST_BBOX:
        params.biggest_box = index;
        break;
      case DIRECTION:
        params.direction = index;
        break;
      case MONOTONE_X:
        params.monotone_x = index;
        break;
      case EUCLIDEAN:
        params.euclidean = index;
        break;
      case SRC_INSIDE:
        params.src_inside = index;
        break;
      case EDGE_INSIDE:
        params.edge_inside = index;
        break;
    }
  }
}

Path_parameters MainWindow::getParameters()
{
  Path_parameters params;
  memset(&params, 0, sizeof(Path_parameters));
  setParameters(initFilterList, 1, params);
  setParameters(secondFilterList, 2, params);
  setParameters(thirdFilterList, 3, params);
  setParameters(fourthFilterList, 4, params);
  setParameters(fifthFilterList, 5, params);
  params.watch_2_edges = watch2EdgesCheck->isChecked();
  return params;
}
