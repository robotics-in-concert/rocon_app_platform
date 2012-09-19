/**
 * @file /include/rocon_projector/main_window.hpp
 *
 * @brief Qt based gui for rocon_projector.
 *
 * @date November 2010
 **/
#ifndef rocon_projector_MAIN_WINDOW_H
#define rocon_projector_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rocon_projector
{

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

  /******************************************
  ** Manual connections
  *******************************************/
	void onBackgroundChangeTriggered(int trigger);



private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	std::vector<QString> picture_paths_;
	unsigned int index_;
};

}  // namespace rocon_projector

#endif // rocon_projector_MAIN_WINDOW_H
