#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	setCentralWidget(&canvas);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
}

MainWindow::~MainWindow() {
}

void MainWindow::onRun() {
	canvas.run();
}

void MainWindow::onStop() {
	canvas.stop();
}