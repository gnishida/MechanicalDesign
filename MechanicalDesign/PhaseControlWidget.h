#ifndef PHASECONTROLWIDGET_H
#define PHASECONTROLWIDGET_H

#include <QDockWidget>
#include "ui_PhaseControlWidget.h"
#include "Kinematics.h"

class MainWindow;

class PhaseControlWidget : public QDockWidget
{
	Q_OBJECT

public:
	Ui::PhaseControlWidget ui;
	std::vector<QSlider*> sliders;
	MainWindow* mainWin;
	std::vector<boost::shared_ptr<kinematics::MechanicalAssembly>> assemblies;

public:
	PhaseControlWidget(MainWindow *parent = 0);
	~PhaseControlWidget();

	void setAssemblies(std::vector<boost::shared_ptr<kinematics::MechanicalAssembly>> assemblies);

public slots:
	void onValueChanged(int value);
};

#endif // PHASECONTROLWIDGET_H
