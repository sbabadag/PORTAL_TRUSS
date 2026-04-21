#include "OpenSeesRunner.h"

#include "LectureTrussSolver.h"
#include "PortalSolver.h"
#include "SectionOptimizer.h"
#include "SteelCatalog.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QTemporaryDir>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace {

constexpr double kN_to_N = 1000.0;

void trussElasticA_I(const PortalFrameInput &in, const SectionOptimizationResult *sizing, const MemberResult &member,
                     double *Aout, double *Iout)
{
    const double A_nom = in.steelArea_m2;
    const double Ic = in.steelInertia_m4;
    const double I_truss_uniform = std::max(Ic * 1.0e-5, 1e-14);

    if (!sizing) {
        *Aout = A_nom;
        *Iout = I_truss_uniform;
        return;
    }

    QString prof;
    switch (member.trussRole) {
    case TrussMemberRole::ChordTop:
        prof = sizing->trussTopChord2xL;
        break;
    case TrussMemberRole::ChordBottom:
    case TrussMemberRole::GussetStrip:
        prof = sizing->trussBottomChord2xL;
        break;
    case TrussMemberRole::EdgePost:
        prof = sizing->trussEdgePost2xL;
        break;
    case TrussMemberRole::Web:
        switch (member.webOptZone) {
        case 0:
            prof = sizing->trussTopChord2xL;
            break;
        case 1:
            prof = sizing->trussWebB2xL;
            break;
        case 2:
            prof = sizing->trussWebC2xL;
            break;
        default:
            prof = sizing->trussWebD2xL;
            break;
        }
        break;
    default:
        *Aout = A_nom;
        *Iout = I_truss_uniform;
        return;
    }

    const auto sec = SteelCatalog::tryGetDoubleAngle2L(prof);
    if (!sec.has_value()) {
        *Aout = A_nom;
        *Iout = I_truss_uniform;
        return;
    }

    const DoubleAngleSection &d = *sec;
    *Aout = d.A_total_m2;
    const double i = std::max(d.i_buckling_m > 0.0 ? d.i_buckling_m : d.i_min_m, 1e-9);
    *Iout = std::max((*Aout) * i * i, 1e-14);
}

/** OpenSees dosyaları çoğunlukla TAB ile ayrılır; split(' ') yalnızca boşlukta böler — kuvvetler sıfır kalırdı. */
QList<double> parseNumericTokens(const QByteArray &line)
{
    QList<double> vals;
    QString s = QString::fromLatin1(line.trimmed());
    s.replace(QLatin1Char('d'), QLatin1Char('e'), Qt::CaseInsensitive);
    const QStringList parts = s.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
    for (const QString &p : parts) {
        bool ok = false;
        const double v = p.toDouble(&ok);
        if (ok) {
            vals.append(v);
        }
    }
    return vals;
}

QString findOpenSeesExecutable()
{
    const QByteArray env = qgetenv("OPENSEES_EXE");
    if (!env.isEmpty()) {
        const QString p = QString::fromLocal8Bit(env);
        if (QFileInfo::exists(p)) {
            return QFileInfo(p).absoluteFilePath();
        }
    }
    // CMake OPENSEES_EXE_TO_BUNDLE ile uygulama klasörüne kopyalanan OpenSees
    const QString appDir = QCoreApplication::applicationDirPath();
#ifdef Q_OS_WIN
    const QString bundled = QDir(appDir).filePath(QStringLiteral("OpenSees.exe"));
#else
    const QString bundled = QDir(appDir).filePath(QStringLiteral("OpenSees"));
#endif
    if (QFileInfo::exists(bundled)) {
        return QFileInfo(bundled).absoluteFilePath();
    }
    return QStringLiteral("OpenSees");
}

bool nodeCoords(const PortalFrameResult &r, int tag, double &x, double &y)
{
    for (const auto &n : r.nodes) {
        if (n.tag == tag) {
            x = n.x;
            y = n.y;
            return true;
        }
    }
    return false;
}

bool isBaseNode(const PortalFrameResult &r, int tag, double W)
{
    double x = 0.0;
    double y = 0.0;
    if (!nodeCoords(r, tag, x, y)) {
        return false;
    }
    return std::abs(y) < 1e-9 && (std::abs(x) < 1e-9 || std::abs(x - W) < 1e-9);
}

/** Uç gusset çubuğunun alt düğümü (aşağı inen uç — mesnet). */
bool isGussetFootNode(const PortalFrameResult &r, int tag)
{
    for (const auto &m : r.members) {
        if (!m.isTruss || m.trussRole != TrussMemberRole::GussetStrip) {
            continue;
        }
        double xi = 0.0;
        double yi = 0.0;
        double xj = 0.0;
        double yj = 0.0;
        if (!nodeCoords(r, m.nodeI, xi, yi) || !nodeCoords(r, m.nodeJ, xj, yj)) {
            continue;
        }
        const int footTag = (yi < yj) ? m.nodeI : m.nodeJ;
        if (footTag == tag) {
            return true;
        }
    }
    return false;
}

QString buildTcl(const PortalFrameInput &in, const PortalFrameResult &geo, const QString &nodePath,
                 const QString &beamPath, const QString &trussPath, double q_roof_design_kN_m,
                 double w_column_design_kN_m, const SectionOptimizationResult *trussPhysicalSizing)
{
#ifdef Q_OS_WIN
    QString np = nodePath;
    QString bp = beamPath;
    QString tp = trussPath;
    np.replace(QLatin1Char('\\'), QLatin1Char('/'));
    bp.replace(QLatin1Char('\\'), QLatin1Char('/'));
    tp.replace(QLatin1Char('\\'), QLatin1Char('/'));
#else
    const QString &np = nodePath;
    const QString &bp = beamPath;
    const QString &tp = trussPath;
#endif
    const double qd = q_roof_design_kN_m;
    const double wd = w_column_design_kN_m;
    const double w_N_m = qd * kN_to_N;
    const double E = in.youngModulus_Pa;
    const double Ac = in.steelArea_m2;
    const double Ic = in.steelInertia_m4;
    const double W = in.spanWidth_m;
    const double Hc = in.columnHeight_m;

    std::map<int, std::pair<double, double>> L;

    for (size_t mi = 0; mi < geo.members.size(); ++mi) {
        const auto &m = geo.members[mi];
        if (!m.isTruss || !m.carriesRoofLineLoad) {
            continue;
        }
        double xi = 0.0;
        double yi = 0.0;
        double xj = 0.0;
        double yj = 0.0;
        if (!nodeCoords(geo, m.nodeI, xi, yi) || !nodeCoords(geo, m.nodeJ, xj, yj)) {
            continue;
        }
        const double Lx = std::abs(xj - xi);
        if (Lx < 1e-12) {
            continue;
        }
        const double half = w_N_m * Lx * 0.5;
        L[m.nodeI].second -= half;
        L[m.nodeJ].second -= half;
    }

    /** Kolon rüzgârı: yayılı wd [kN/m] boyunca — uçta tek nokta wd·H yerine eleLoad beamUniform (taban momenti wd·H²/2 ile uyumlu). */
    const double w_wind_N_m = wd * kN_to_N;
    QString columnEleTags;
    for (const auto &m : geo.members) {
        if (m.isTruss) {
            continue;
        }
        if (!columnEleTags.isEmpty()) {
            columnEleTags += QLatin1Char(' ');
        }
        columnEleTags += QString::number(m.tag);
    }

    QString tcl;
    tcl += QStringLiteral("wipe\n");
    tcl += QStringLiteral("model basic -ndm 2 -ndf 3\n");
    tcl += QStringLiteral("uniaxialMaterial Elastic 1 %1\n").arg(E, 0, 'g', 17);

    const int rzFix = (in.columnBaseSupport == ColumnBaseSupport::Pinned) ? 0 : 1;
    for (const auto &n : geo.nodes) {
        tcl += QStringLiteral("node %1 %2 %3\n").arg(n.tag).arg(n.x, 0, 'g', 17).arg(n.y, 0, 'g', 17);
        if (isBaseNode(geo, n.tag, W) || isGussetFootNode(geo, n.tag)) {
            tcl += QStringLiteral("fix %1 1 1 %2\n").arg(n.tag).arg(rzFix);
        }
    }

    tcl += QStringLiteral("geomTransf Linear 1\n");

    for (const auto &m : geo.members) {
        const int eid = m.tag;
        if (!m.isTruss) {
            tcl += QStringLiteral("element elasticBeamColumn %1 %2 %3 %4 %5 %6 1\n")
                       .arg(eid)
                       .arg(m.nodeI)
                       .arg(m.nodeJ)
                       .arg(Ac, 0, 'g', 17)
                       .arg(E, 0, 'g', 17)
                       .arg(Ic, 0, 'g', 17);
        } else {
            double Atr = in.steelArea_m2;
            double Itr = std::max(Ic * 1.0e-5, 1e-14);
            trussElasticA_I(in, trussPhysicalSizing, m, &Atr, &Itr);
            tcl += QStringLiteral("element elasticBeamColumn %1 %2 %3 %4 %5 %6 1\n")
                       .arg(eid)
                       .arg(m.nodeI)
                       .arg(m.nodeJ)
                       .arg(Atr, 0, 'g', 17)
                       .arg(E, 0, 'g', 17)
                       .arg(Itr, 0, 'g', 17);
        }
    }

    tcl += QStringLiteral("timeSeries Linear 1\n");
    tcl += QStringLiteral("pattern Plain 1 1 {\n");
    const bool windEle = std::abs(w_wind_N_m) > 1e-12 && !columnEleTags.isEmpty();
    if (L.empty() && !windEle) {
        tcl += QStringLiteral("  load 1 0.0 0.0 0.0\n");
    } else {
        for (const auto &kv : L) {
            tcl += QStringLiteral("  load %1 %2 %3 0.0\n")
                       .arg(kv.first)
                       .arg(kv.second.first, 0, 'g', 17)
                       .arg(kv.second.second, 0, 'g', 17);
        }
        if (windEle) {
            /** geomTransf Linear: kolon +Y boyunca iken yerel y = −X; +X yönünde yayılı kuvvet için Wy = −w. */
            tcl += QStringLiteral("  eleLoad -ele %1 -type -beamUniform %2\n")
                       .arg(columnEleTags)
                       .arg(-w_wind_N_m, 0, 'g', 17);
        }
    }
    tcl += QStringLiteral("}\n");

    QString nodeList;
    for (const auto &n : geo.nodes) {
        if (!nodeList.isEmpty()) {
            nodeList += ' ';
        }
        nodeList += QString::number(n.tag);
    }

    /** GussetStrip: Tcl’de elasticBeamColumn (6 yerel bileşen); basicForces yalnızca 3 değer — makas kaydına koymak tüm kuvvet satırını kaydırır. */
    auto usesBeamForceRecorder = [](const MemberResult &m) {
        return !m.isTruss || m.trussRole == TrussMemberRole::GussetStrip;
    };
    auto usesTrussForceRecorder = [](const MemberResult &m) {
        return m.isTruss && m.trussRole != TrussMemberRole::GussetStrip;
    };

    QString beamEles;
    QString trussEles;
    for (const auto &m : geo.members) {
        if (usesBeamForceRecorder(m)) {
            if (!beamEles.isEmpty()) {
                beamEles += ' ';
            }
            beamEles += QString::number(m.tag);
        }
        if (usesTrussForceRecorder(m)) {
            if (!trussEles.isEmpty()) {
                trussEles += ' ';
            }
            trussEles += QString::number(m.tag);
        }
    }

    // RCM + FullGeneral: BandGeneral bazen kötü sıralamada sıfır pivot gösterir; küçük sistemlerde güvenli.
    tcl += QStringLiteral("constraints Plain\n");
    tcl += QStringLiteral("numberer RCM\n");
    tcl += QStringLiteral("system FullGeneral\n");
    tcl += QStringLiteral("test NormUnbalance 1.0e-9 10 0\n");
    tcl += QStringLiteral("algorithm Linear\n");
    tcl += QStringLiteral("integrator LoadControl 1.0\n");

    // Tcl: Windows'ta "C:\Users\..." içinde \U kaçışı bozar; süslü parantez + / yolu güvenli.
    tcl += QStringLiteral("recorder Node -file {%1} -time -node %2 -dof 1 2 3 disp\n")
               .arg(np, nodeList);

    if (!beamEles.isEmpty()) {
        // OpenSees: seçenek adı "localForce" (başında tire yok); "-localForce" kaydı bozar.
        tcl += QStringLiteral("recorder Element -file {%1} -time -ele %2 localForce\n")
                   .arg(bp, beamEles);
    }
    if (!trussEles.isEmpty()) {
        // basicForces: q(0)=N, q(1)=M1, q(2)=M2 — tek eksenel kuvvet (makas için doğru).
        // localForce uçlarda N ve -N verir; yanlış uç seçimi basıncı çekme sanabilir.
        tcl += QStringLiteral("recorder Element -file {%1} -time -ele %2 basicForces\n")
                   .arg(tp, trussEles);
    }

    tcl += QStringLiteral("analysis Static\n");
    tcl += QStringLiteral("analyze 1\n");
    return tcl;
}

bool parseNodeDisp(const QString &path, PortalFrameResult &geo, QString *err)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (err) {
            *err = QStringLiteral("OpenSees yer değiştirme dosyası yok: %1").arg(path);
        }
        return false;
    }
    const QByteArray data = f.readAll();
    f.close();
    const QList<QByteArray> lines = data.split('\n');
    QByteArray lastLine;
    for (int i = lines.size() - 1; i >= 0; --i) {
        const QByteArray t = lines.at(i).trimmed();
        if (!t.isEmpty() && t[0] != '#') {
            lastLine = t;
            break;
        }
    }
    if (lastLine.isEmpty()) {
        if (err) {
            *err = QStringLiteral("OpenSees çıktısı boş.");
        }
        return false;
    }

    const QList<double> vals = parseNumericTokens(lastLine);

    const int need = 1 + static_cast<int>(geo.nodes.size()) * 3;
    if (vals.size() < need) {
        if (err) {
            *err = QStringLiteral("Yer değiştirme sütun sayısı uyuşmuyor (beklenen %1, gelen %2).")
                       .arg(need)
                       .arg(vals.size());
        }
        return false;
    }

    for (size_t i = 0; i < geo.nodes.size(); ++i) {
        geo.nodes[i].ux = vals[1 + static_cast<int>(i) * 3];
        geo.nodes[i].uy = vals[2 + static_cast<int>(i) * 3];
        geo.nodes[i].rz = vals[3 + static_cast<int>(i) * 3];
    }
    return true;
}

void parseBeamForcesLine(const QList<double> &vals, PortalFrameResult &geo)
{
    if (vals.size() < 2) {
        return;
    }
    int idx = 1;
    for (auto &m : geo.members) {
        const bool beamRec = !m.isTruss || m.trussRole == TrussMemberRole::GussetStrip;
        if (beamRec) {
            if (idx + 5 < static_cast<int>(vals.size())) {
                const double n1 = vals[idx];
                const double n2 = vals[idx + 3];
                m.axial_N = (std::abs(n1) >= std::abs(n2)) ? n1 : n2;
                m.moment_i_Nm = vals[idx + 2];
                m.moment_j_Nm = vals[idx + 5];
            }
            idx += 6;
        }
    }
}

void parseTrussForcesLine(const QList<double> &vals, PortalFrameResult &geo)
{
    if (vals.size() < 2) {
        return;
    }
    int idx = 1;
    for (auto &m : geo.members) {
        if (m.isTruss && m.trussRole != TrussMemberRole::GussetStrip) {
            if (idx + 2 < static_cast<int>(vals.size())) {
                m.axial_N = vals[idx];
                m.moment_i_Nm = vals[idx + 1];
                m.moment_j_Nm = vals[idx + 2];
            }
            idx += 3;
        }
    }
}

bool readLastNumberLine(const QString &path, QList<double> *out)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }
    const QByteArray data = f.readAll();
    f.close();
    const QList<QByteArray> lines = data.split('\n');
    QByteArray lastLine;
    for (int i = lines.size() - 1; i >= 0; --i) {
        const QByteArray t = lines.at(i).trimmed();
        if (!t.isEmpty() && t[0] != '#') {
            lastLine = t;
            break;
        }
    }
    if (lastLine.isEmpty()) {
        return false;
    }
    *out = parseNumericTokens(lastLine);
    return !out->isEmpty();
}

} // namespace

bool runOpenSeesStaticAnalysis(const PortalFrameInput &input, PortalFrameResult &ioResult,
                               double q_roof_design_kN_m, double w_column_design_kN_m, QString *errorOut,
                               const SectionOptimizationResult *trussPhysicalSizing)
{
    if (ioResult.nodes.empty()) {
        PortalSolver::buildPortalGeometry(input, ioResult);
    }

    QTemporaryDir tmpDir;
    if (!tmpDir.isValid()) {
        if (errorOut) {
            *errorOut = QStringLiteral("Geçici klasör oluşturulamadı.");
        }
        return false;
    }

    const QString nodeOut = tmpDir.path() + QStringLiteral("/nodes_disp.txt");
    const QString beamOut = tmpDir.path() + QStringLiteral("/forces_beam.txt");
    const QString trussOut = tmpDir.path() + QStringLiteral("/forces_truss.txt");
    const QString tclPath = tmpDir.path() + QStringLiteral("/portal_model.tcl");

    const QString tcl = buildTcl(input, ioResult, nodeOut, beamOut, trussOut, q_roof_design_kN_m,
                                 w_column_design_kN_m, trussPhysicalSizing);
    {
        QFile tf(tclPath);
        if (!tf.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if (errorOut) {
                *errorOut = QStringLiteral("Tcl dosyası yazılamadı.");
            }
            return false;
        }
        tf.write(tcl.toUtf8());
        tf.close();
    }

    const QString exe = findOpenSeesExecutable();
    QProcess proc;
    proc.setProgram(exe);
    proc.setArguments({tclPath});
    proc.setWorkingDirectory(tmpDir.path());
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    // Berkeley OpenSees: yürütülebilir dosya yanında lib/tcl8.6/init.tcl olmalı; aksi halde Tcl başlamaz.
    const QString exeDir = QFileInfo(exe).absolutePath();
    const QString initTcl = QDir(exeDir).filePath(QStringLiteral("lib/tcl8.6/init.tcl"));
    if (QFileInfo::exists(initTcl)) {
        const QString tclLib = QDir(exeDir).filePath(QStringLiteral("lib/tcl8.6"));
        env.insert(QStringLiteral("TCL_LIBRARY"), QFileInfo(tclLib).absoluteFilePath());
    }
    proc.setProcessEnvironment(env);
    proc.setProcessChannelMode(QProcess::MergedChannels);
    proc.start();
    if (!proc.waitForFinished(120000)) {
        proc.kill();
        if (errorOut) {
            *errorOut = QStringLiteral(
                "OpenSees başlatılamadı veya zaman aşımı. "
                "https://opensees.github.io/OpenSees/ üzerinden indirip OPENSEES_EXE ortam değişkenine tam yolu "
                "yazın veya OpenSees.exe’yi PATH’e ekleyin.");
        }
        return false;
    }

    const QByteArray logOut = proc.readAllStandardOutput() + proc.readAllStandardError();

    if (proc.exitCode() != 0) {
        if (errorOut) {
            *errorOut = QStringLiteral("OpenSees çıkış kodu %1:\n%2")
                            .arg(proc.exitCode())
                            .arg(QString::fromUtf8(logOut.left(4000)));
        }
        return false;
    }

    QString perr;
    if (!parseNodeDisp(nodeOut, ioResult, &perr)) {
        if (errorOut) {
            *errorOut = perr + QStringLiteral("\n\n— OpenSees çıktısı —\n")
                            + QString::fromUtf8(logOut.left(6000));
        }
        return false;
    }

    QList<double> beamVals;
    if (readLastNumberLine(beamOut, &beamVals)) {
        parseBeamForcesLine(beamVals, ioResult);
    }
    QList<double> trussVals;
    if (readLastNumberLine(trussOut, &trussVals)) {
        parseTrussForcesLine(trussVals, ioResult);
    }

    QString lecSum;
    if (LectureTrussSolver::compareAxialVsOpenSees(input, ioResult, q_roof_design_kN_m, w_column_design_kN_m,
                                                   trussPhysicalSizing, &lecSum, nullptr)) {
        ioResult.lectureTrussComparisonNote = lecSum;
    } else if (!lecSum.isEmpty()) {
        ioResult.lectureTrussComparisonNote = lecSum;
    }

    (void)logOut;
    return true;
}
