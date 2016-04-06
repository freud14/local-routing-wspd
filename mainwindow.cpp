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

// for viewportsBbox
#include <CGAL/Qt/utility.h>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow() :
    DemosMainWindow(), s(2.0), wspd(2, 2.0)
{
  setupUi(this);

  set_path_field();
  QObject::connect(findPathButton, SIGNAL (released()),this, SLOT (findPath()));
  QObject::connect(erasePathButton, SIGNAL (released()),this, SLOT (erasePath()));
  QObject::connect(swapButton, SIGNAL (released()),this, SLOT (swapSourceDest()));
  QObject::connect(randomTestsButton, SIGNAL (released()),this, SLOT (randomTests()));

  pgi = new CGAL::Qt::PointsGraphicsItem<Point_vector>(&points);
  QObject::connect(this, SIGNAL(changed()), pgi, SLOT(modelChanged()));
  pgi->setVerticesPen(QPen(Qt::black, 10, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  scene.addItem(pgi);

  pngi = new PointNumbersGraphicsItem<Point_vector>(&points, &path_found);
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
  this->actionInsertPoint->setChecked(true);
  this->actionShowWspd->setChecked(true);
  wspd_item->setDrawWspd(true);
  this->actionShowBoundingBoxes->setChecked(false);
  wspd_item->setDrawBoundingBoxes(false);

  numberPointsTests->setMaximum(10000);
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
      reset_wspd();
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
  reset_wspd();
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
  reset_wspd();
  // default cursor
  QApplication::setOverrideCursor(Qt::ArrowCursor);
  Q_EMIT( changed());
}

void MainWindow::on_actionSetSeparationRatio_triggered() {
  s =
    QInputDialog::getDouble(this,
                             tr("Separation ratio"),
                             tr("Enter the separation ratio"), s, 0.0);
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
  Point_2 p;
  while(ifs >> p) {
    points.push_back(p);
  }
  reset_wspd();

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
  path_found = wspd.find_path(textFrom->value(), textTo->value(), out, true);
  out << "----------------------------------------------" << std::endl;
  results->setText(out.str().c_str());
  results->verticalScrollBar()->triggerAction(QAbstractSlider::SliderToMaximum);
  erasePathButton->setEnabled(true);
  Q_EMIT( changed());
}

void MainWindow::erasePath()
{
  path_found.clear();
  erasePathButton->setEnabled(false);
  Q_EMIT( changed());
}

void MainWindow::randomTests()
{
  //wspd.display_wspd(out);
  QRectF rect = CGAL::Qt::viewportsBbox(&scene);
  Iso_rectangle_2 isor = convert(rect);
  CGAL::Random_points_in_iso_rectangle_2<Point_2> pg(isor.min(), isor.max());

  int n = numberPointsTests->value();
  int nbTest = 10;
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
          std::vector<int> path = wspd.find_path(from, to);
          if(!wspd.verify_algo_induction_proof(path)) {
            path_found = wspd.find_path(from, to, out, true);

            counter_example_found = true;
            goto end_loops;
          }
          else {
            const Well_separated_pair& pair = wspd.get_wsp(from, to);
            const Node* node = pair.first;
            if(pair.second->bounding_box().bounded_side(points[to]) != -1) {
              node = pair.second;
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
    set_path_field();
    textFrom->setValue(from);
    textTo->setValue(to);
  }
  else {
    reset_wspd();
    out << "Test passed!" << std::endl;
  }
  Q_EMIT( changed());

  out << "----------------------------------------------" << std::endl;
  results->setText(out.str().c_str());
  results->verticalScrollBar()->triggerAction(QAbstractSlider::SliderToMaximum);
}

void
MainWindow::reset_wspd()
{
  path_found.clear();
  wspd.set(2, points.begin(), points.end());
  set_path_field();
}

void
MainWindow::set_path_field()
{
  if(points.size() != 0) {
    textFrom->setMaximum(points.size()-1);
    textFrom->setEnabled(true);
    textTo->setMaximum(points.size()-1);
    textTo->setEnabled(true);
    swapButton->setEnabled(true);
    findPathButton->setEnabled(true);
  }
  else {
    textFrom->setMaximum(0);
    textFrom->setEnabled(false);
    textTo->setMaximum(0);
    textTo->setEnabled(false);
    swapButton->setEnabled(false);
    findPathButton->setEnabled(false);
  }
  if(path_found.size() != 0) {
    erasePathButton->setEnabled(true);
  }
  else {
    erasePathButton->setEnabled(false);
  }
}
