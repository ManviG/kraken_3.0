#include "gui_2.h"
#include <QApplication>
#include <iostream>
#include <fstream>
#include <QFile>
#include <QtXml>
#include <QtCore/QMessageLogger>

using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    gui_2 w;
    w.show();
    gui_2 xml;

    return a.exec();
}
