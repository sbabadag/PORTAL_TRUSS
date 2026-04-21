#include <QApplication>
#include <QCoreApplication>
#include <QString>

#include "MainWindow.h"
#include "PortalSolver.h"
#include "SectionOptimizer.h"

#include <cstdio>

int main(int argc, char *argv[])
{
    if (argc >= 2 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--self-check")) {
        QCoreApplication app(argc, argv);
        const QString report = runPortalSelfCheckDefaultInput();
        const QByteArray utf8 = report.toUtf8();
        fwrite(utf8.constData(), 1, static_cast<size_t>(utf8.size()), stdout);
        fputc('\n', stdout);
        return report.startsWith(QStringLiteral("FAIL")) ? 1 : 0;
    }

    QApplication app(argc, argv);

    qRegisterMetaType<PortalFrameResult>("PortalFrameResult");

    MainWindow w;
    w.resize(1100, 780);
    w.show();
    return app.exec();
}
