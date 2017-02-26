/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowClass
{
public:
    QAction *actionExit;
    QAction *actionRun;
    QAction *actionStop;
    QAction *actionShowAssemblies;
    QAction *actionShowAll;
    QAction *actionShowLinks;
    QAction *actionShowBodies;
    QAction *actionOpen;
    QAction *actionSave;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuTool;
    QMenu *menuOptions;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindowClass)
    {
        if (MainWindowClass->objectName().isEmpty())
            MainWindowClass->setObjectName(QStringLiteral("MainWindowClass"));
        MainWindowClass->resize(800, 800);
        actionExit = new QAction(MainWindowClass);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionRun = new QAction(MainWindowClass);
        actionRun->setObjectName(QStringLiteral("actionRun"));
        actionStop = new QAction(MainWindowClass);
        actionStop->setObjectName(QStringLiteral("actionStop"));
        actionShowAssemblies = new QAction(MainWindowClass);
        actionShowAssemblies->setObjectName(QStringLiteral("actionShowAssemblies"));
        actionShowAssemblies->setCheckable(true);
        actionShowAll = new QAction(MainWindowClass);
        actionShowAll->setObjectName(QStringLiteral("actionShowAll"));
        actionShowLinks = new QAction(MainWindowClass);
        actionShowLinks->setObjectName(QStringLiteral("actionShowLinks"));
        actionShowLinks->setCheckable(true);
        actionShowBodies = new QAction(MainWindowClass);
        actionShowBodies->setObjectName(QStringLiteral("actionShowBodies"));
        actionShowBodies->setCheckable(true);
        actionOpen = new QAction(MainWindowClass);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionSave = new QAction(MainWindowClass);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        centralWidget = new QWidget(MainWindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MainWindowClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 800, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuTool = new QMenu(menuBar);
        menuTool->setObjectName(QStringLiteral("menuTool"));
        menuOptions = new QMenu(menuBar);
        menuOptions->setObjectName(QStringLiteral("menuOptions"));
        MainWindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindowClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindowClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuTool->menuAction());
        menuBar->addAction(menuOptions->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuTool->addAction(actionRun);
        menuTool->addAction(actionStop);
        menuOptions->addAction(actionShowAll);
        menuOptions->addSeparator();
        menuOptions->addAction(actionShowAssemblies);
        menuOptions->addAction(actionShowLinks);
        menuOptions->addAction(actionShowBodies);

        retranslateUi(MainWindowClass);

        QMetaObject::connectSlotsByName(MainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowClass)
    {
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "Mechanical Design", 0));
        actionExit->setText(QApplication::translate("MainWindowClass", "Exit", 0));
        actionRun->setText(QApplication::translate("MainWindowClass", "Run", 0));
        actionStop->setText(QApplication::translate("MainWindowClass", "Stop", 0));
        actionShowAssemblies->setText(QApplication::translate("MainWindowClass", "Show Assemblies", 0));
        actionShowAll->setText(QApplication::translate("MainWindowClass", "Show All", 0));
        actionShowLinks->setText(QApplication::translate("MainWindowClass", "Show Links", 0));
        actionShowBodies->setText(QApplication::translate("MainWindowClass", "Show Bodies", 0));
        actionOpen->setText(QApplication::translate("MainWindowClass", "Open", 0));
        actionOpen->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+O", 0));
        actionSave->setText(QApplication::translate("MainWindowClass", "Save", 0));
        actionSave->setShortcut(QApplication::translate("MainWindowClass", "Ctrl+S", 0));
        menuFile->setTitle(QApplication::translate("MainWindowClass", "File", 0));
        menuTool->setTitle(QApplication::translate("MainWindowClass", "Tool", 0));
        menuOptions->setTitle(QApplication::translate("MainWindowClass", "Options", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
