/********************************************************************************
** Form generated from reading UI file 'PhaseControlWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PHASECONTROLWIDGET_H
#define UI_PHASECONTROLWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PhaseControlWidget
{
public:
    QWidget *widget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;

    void setupUi(QDockWidget *PhaseControlWidget)
    {
        if (PhaseControlWidget->objectName().isEmpty())
            PhaseControlWidget->setObjectName(QStringLiteral("PhaseControlWidget"));
        PhaseControlWidget->resize(479, 122);
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        verticalLayoutWidget = new QWidget(widget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(-1, -1, 481, 101));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        PhaseControlWidget->setWidget(widget);

        retranslateUi(PhaseControlWidget);

        QMetaObject::connectSlotsByName(PhaseControlWidget);
    } // setupUi

    void retranslateUi(QDockWidget *PhaseControlWidget)
    {
        PhaseControlWidget->setWindowTitle(QApplication::translate("PhaseControlWidget", "Phase Control", 0));
    } // retranslateUi

};

namespace Ui {
    class PhaseControlWidget: public Ui_PhaseControlWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PHASECONTROLWIDGET_H
