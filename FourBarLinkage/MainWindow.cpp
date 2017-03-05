#include "MainWindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);

	setCentralWidget(&canvas);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionStepForward, SIGNAL(triggered()), this, SLOT(onStepForward()));
	connect(ui.actionStepBackward, SIGNAL(triggered()), this, SLOT(onStepBackward()));
	connect(ui.actionIncreaseSpeed, SIGNAL(triggered()), this, SLOT(onIncreaseSpeed()));
	connect(ui.actionDecreaseSpeed, SIGNAL(triggered()), this, SLOT(onDecreaseSpeed()));
}

MainWindow::~MainWindow() {
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas.open(filename);
}

void MainWindow::onSave() {
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	canvas.save(filename);
}

void MainWindow::onRun() {
	canvas.run();
}

void MainWindow::onStop() {
	canvas.stop();
}

void MainWindow::onStepForward() {
	canvas.stepForward();
}

void MainWindow::onStepBackward() {
	canvas.stepBackward();
}

void MainWindow::onIncreaseSpeed() {
	canvas.speed *= 2;
}

void MainWindow::onDecreaseSpeed() {
	canvas.speed *= 0.5;
}