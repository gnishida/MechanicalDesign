#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include "Canvas.h"
#include "PhaseControlWidget.h"

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	Ui::MainWindowClass ui;
	Canvas canvas;
	PhaseControlWidget* phaseControlWidget;

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();


public slots:
	void onOpen();
	void onSave();
	void onRun();
	void onStop();
	void onPhaseControl();
	void onShowAll();
	void onShowChanged();
};

#endif // MAINWINDOW_H
