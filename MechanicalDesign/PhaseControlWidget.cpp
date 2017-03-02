#include "PhaseControlWidget.h"
#include <iostream>
#include <QSlider>
#include "MainWindow.h"

PhaseControlWidget::PhaseControlWidget(MainWindow *parent) : QDockWidget("Phase Control") {
	mainWin = parent;
	ui.setupUi(this);
}

PhaseControlWidget::~PhaseControlWidget() {
}

void PhaseControlWidget::setAssemblies(std::vector<boost::shared_ptr<kinematics::MechanicalAssembly>> assemblies) {
	// clear the existing sliders
	for (int i = 0; i < sliders.size(); ++i) {
		ui.verticalLayout->removeWidget(sliders[i]);
		delete sliders[i];
	}
	sliders.clear();

	this->assemblies = assemblies;

	// add sliders
	for (int i = 0; i < assemblies.size(); ++i) {
		QSlider* slider = new QSlider(Qt::Horizontal, this);
		slider->setMinimum(0);
		slider->setMaximum(628);
		slider->setValue(assemblies[i]->phase * 100);
		ui.verticalLayout->addWidget(slider);
		sliders.push_back(slider);

		connect(slider, SIGNAL(valueChanged(int)), this, SLOT(onValueChanged(int)));
	}
}

void PhaseControlWidget::onValueChanged(int value) {
	for (int i = 0; i < sliders.size(); ++i) {
		float target_phase = (float)sliders[i]->value() * 0.01;

		assemblies[i]->forward(target_phase - assemblies[i]->phase);
	}
	mainWin->canvas.kinematics.forwardKinematics();
	mainWin->canvas.update();
}