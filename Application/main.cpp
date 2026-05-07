#include <QtWidgets/QApplication>
#include "MainWindow.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication qtapplication(argc, argv);

	MainWindow mainWindow;
	mainWindow.show();

    return qtapplication.exec();
}


