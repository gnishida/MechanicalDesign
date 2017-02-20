#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);
	ui.actionShowAssemblies->setChecked(true);
	ui.actionShowLinks->setChecked(true);
	ui.actionShowBodies->setChecked(true);

	setCentralWidget(&canvas);

	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionShowAll, SIGNAL(triggered()), this, SLOT(onShowAll()));
	connect(ui.actionShowAssemblies, SIGNAL(triggered()), this, SLOT(onShowChanged()));
	connect(ui.actionShowLinks, SIGNAL(triggered()), this, SLOT(onShowChanged()));
	connect(ui.actionShowBodies, SIGNAL(triggered()), this, SLOT(onShowChanged()));
}

MainWindow::~MainWindow() {
}

void MainWindow::onRun() {
	canvas.run();
}

void MainWindow::onStop() {
	canvas.stop();
}

void MainWindow::onShowAll() {
	// update the menu
	ui.actionShowAssemblies->setChecked(true);
	ui.actionShowLinks->setChecked(true);
	ui.actionShowBodies->setChecked(true);

	canvas.showAssemblies(true);
	canvas.showLinks(true);
	canvas.showBodies(true);
}

void MainWindow::onShowChanged() {
	canvas.showAssemblies(ui.actionShowAssemblies->isChecked());
	canvas.showLinks(ui.actionShowLinks->isChecked());
	canvas.showBodies(ui.actionShowBodies->isChecked());
}

