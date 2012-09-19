/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rocon_projector/main_window.hpp"

 #include <QSplashScreen>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rocon_projector {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
  QObject::connect(&qnode, SIGNAL(changeBackgroundTrigger(int)), this, SLOT(onBackgroundChangeTriggered(int))); // qApp is a global variable for the application
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close())); // qApp is a global variable for the application
  //ReadSettings();
  QPoint pos = QPoint(200, 200);
  QWidget::showMaximized();
  //QWidget::showFullScreen();
  //QSize size = QSize(200, 150);
  //resize(size);
  move(pos);

  /*********************
  ** Auto Start
  **********************/
  if ( !qnode.init() )
    showNoMasterMessage();

//  QApplication app(argc, argv);
//  QPixmap pixmap(":/images/icon.png");
//  QSplashScreen splash(pixmap);
//  splash.show();
//  app.processEvents();
  //qApp->processEvents();

//  QMainWindow window;
//  window.show();
//  splash.finish(&window);
//  return app.exec();

  //QApplication a(argc, argv);
  //showMaximized();
//  setFixedSize( QSize(1920, 1080) );
//  QGraphicsScene scene;
//  QGraphicsView view(&scene);
//  QGraphicsPixmapItem item(QPixmap(":/images/icon.png"));
//  scene.addItem(&item);
//  view.show();
  //qApp->exec();

  /*
   * Let's add some pictures
   */
  picture_paths_.push_back(":/images/rocon-logo-grey-window.png"); // ui starts with this as default
  picture_paths_.push_back(":/images/dk04.jpg");
  picture_paths_.push_back(":/images/dk05.jpg");
  picture_paths_.push_back(":/images/dk06.jpg");
  picture_paths_.push_back(":/images/dk07.jpg");
  picture_paths_.push_back(":/images/dk08.jpg");
  index_ = 0;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
  close();
}

void MainWindow::onBackgroundChangeTriggered(int trigger)
{
  QWidget::showFullScreen();
  if ( trigger == 1 || trigger == -1 )
  {
    if( picture_paths_.size() != 0 )
    {
      if ( trigger == 1 )
        std::cout << "Background change triggered. Changing to next background." << std::endl;
      else
        std::cout << "Background change triggered. Changing to last background." << std::endl;

      if( index_ == 0 && trigger == -1 )
        index_ = picture_paths_.size() - 1;
      else if( (index_ + trigger)>= picture_paths_.size() )
        index_ = 0;
      else
        index_ = index_ + trigger;

      std::cout << "New background will be: " << picture_paths_[index_].toStdString() << std::endl;
      QPixmap pixmap(picture_paths_[index_]);
      ui.label->setPixmap(pixmap.scaled(ui.label->width(),ui.label->height(),Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    else
      std::cout << "No picture paths specified! Hence, no new background is set." << std::endl;
  }
  else
    std::cout << "Wrong trigger value sent. No new background is set." << std::endl;
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Rocon Projector App 1.00</h2><p>Copyright Yujin Robot</p><p>This app allows changing the background of the screen through ROS."));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings()
{
  QSettings settings("Qt-Ros Package", "rocon_projector");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings()
{
  QSettings settings("Qt-Ros Package", "rocon_projector");
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace rocon_projector

