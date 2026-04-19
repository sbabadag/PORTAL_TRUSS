#include <QApplication>

#include "MainWindow.h"
#include "PortalSolver.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    qRegisterMetaType<PortalFrameResult>("PortalFrameResult");

    MainWindow w;
    w.resize(1100, 700);
    w.show();
    return app.exec();
}
