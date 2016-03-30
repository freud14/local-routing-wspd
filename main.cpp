#include "mainwindow.h"
#include "mainwindow.moc"
#include <QApplication>
#include <CGAL/Qt/resources.h>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  CGAL_QT_INIT_RESOURCES;

  MainWindow w;
  w.show();
  if(argc == 2) {
    w.open(argv[1]);
  }

  return a.exec();
}
