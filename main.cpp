#include <QApplication>
#include <QCoreApplication>
#include <QString>

#include "MainWindow.h"
#include "PortalSolver.h"
#include "SectionOptimizer.h"

#include <cstdio>

namespace {

PortalFrameInput uiDefaultInput()
{
    PortalFrameInput in;
    in.spanWidth_m = 27.7;
    in.columnHeight_m = 9.0;
    in.apexHeight_m = PortalSolver::trussApexHeight_m(in);
    in.trussAxisSpacingY_m = 7.65;
    in.trussPanelsPerSide = 15;
    in.steelArea_m2 = 0.015;
    in.steelInertia_m4 = 5.0e-4;
    in.youngModulus_Pa = 200e9;
    in.purlinCladding_kN_per_m2 = 0.05;
    in.dl2_kN_per_m2 = 0.2;
    in.sk_kN_per_m2 = 1.54;
    in.wl_kN_per_m2 = 0.64;
    in.fy_MPa = 235.0;
    in.columnFamilyIndex = 1; // UI default: HEB
    in.trussMemberSectionForm = 0; // UI default: 2xL Warren
    in.columnBaseSupport = ColumnBaseSupport::Fixed;
    in.columnBucklingKy = 2.0;
    in.columnBucklingKz = 2.0;
    in.columnBucklingCurveOrdinal = 2;
    in.trussBucklingCurveOrdinal = 2;
    in.columnLtbCurveOrdinal = 2;
    in.columnLateralBraceFromTrussGeometry = true;
    return in;
}

void printUtf8Line(const QString &text)
{
    const QByteArray utf8 = text.toUtf8();
    fwrite(utf8.constData(), 1, static_cast<size_t>(utf8.size()), stdout);
    fputc('\n', stdout);
}

} // namespace

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
    if (argc >= 2 && QString::fromLocal8Bit(argv[1]) == QStringLiteral("--compare-ui-default")) {
        QCoreApplication app(argc, argv);
        const PortalFrameInput in = uiDefaultInput();
        const PortalFrameResult r = PortalSolver().analyze(in);
        printUtf8Line(r.errorMessage);
        if (!r.lectureTrussComparisonNote.isEmpty()) {
            printUtf8Line(r.lectureTrussComparisonNote);
        }
        return r.ok ? 0 : 1;
    }

    QApplication app(argc, argv);

    qRegisterMetaType<PortalFrameResult>("PortalFrameResult");

    MainWindow w;
    w.resize(1100, 780);
    w.show();
    return app.exec();
}
