#include "MainWindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
	ui.setupUi(this);
	ui.actionShowAssemblies->setChecked(true);
	ui.actionShowLinks->setChecked(true);
	ui.actionShowBodies->setChecked(true);

	setCentralWidget(&canvas);
	phaseControlWidget = new PhaseControlWidget(this);

	connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui.actionRun, SIGNAL(triggered()), this, SLOT(onRun()));
	connect(ui.actionStop, SIGNAL(triggered()), this, SLOT(onStop()));
	connect(ui.actionPhaseControl, SIGNAL(triggered()), this, SLOT(onPhaseControl()));
	connect(ui.actionShowAll, SIGNAL(triggered()), this, SLOT(onShowAll()));
	connect(ui.actionShowAssemblies, SIGNAL(triggered()), this, SLOT(onShowChanged()));
	connect(ui.actionShowLinks, SIGNAL(triggered()), this, SLOT(onShowChanged()));
	connect(ui.actionShowBodies, SIGNAL(triggered()), this, SLOT(onShowChanged()));
}

MainWindow::~MainWindow() {
}

void MainWindow::onOpen() {
	QString filename = QFileDialog::getOpenFileName(this, tr("Open Design file..."), "", tr("Design Files (*.xml)"));
	if (filename.isEmpty()) return;

	try {
		canvas.open(filename);
	}
	catch (char* ex) {
		QMessageBox::warning(this, "Error message", ex);
	}
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

void MainWindow::onPhaseControl() {
	phaseControlWidget->setAssemblies(canvas.kinematics.assemblies);
	phaseControlWidget->show();
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

