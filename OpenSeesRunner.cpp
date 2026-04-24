#include "OpenSeesRunner.h"

#include "LectureTrussSolver.h"
#include "PortalSolver.h"
#include "SectionOptimizer.h"
#include "SteelCatalog.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QTemporaryDir>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <utility>

#ifdef Q_OS_WIN
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {

constexpr double kN_to_N = 1000.0;

bool stderrIsConsole()
{
#ifdef Q_OS_WIN
    return _isatty(_fileno(stderr)) != 0;
#else
    return isatty(fileno(stderr)) != 0;
#endif
}

/**
 * OpenSees aşamaları ve süreç çıktısını stderr’e yaz (cmd/PowerShell’den çalıştırınca görünür).
 * PORTAL_OPENSEES_CONSOLE=1|0 — stderr TTY değilken zorla aç/kapat.
 */
bool openSeesConsoleEcho()
{
    const QByteArray raw = qgetenv("PORTAL_OPENSEES_CONSOLE");
    if (!raw.isEmpty()) {
        const QString s = QString::fromLatin1(raw.trimmed()).toUpper();
        if (s == QStringLiteral("1") || s == QStringLiteral("TRUE") || s == QStringLiteral("YES")
            || s == QStringLiteral("ON")) {
            return true;
        }
        if (s == QStringLiteral("0") || s == QStringLiteral("FALSE") || s == QStringLiteral("NO")
            || s == QStringLiteral("OFF")) {
            return false;
        }
    }
    return stderrIsConsole();
}

void writeUtf8LineToStderr(const QString &line)
{
    const QByteArray b = line.toUtf8() + '\n';
    fwrite(b.constData(), 1, static_cast<size_t>(b.size()), stderr);
    fflush(stderr);
}

void writeUtf8ChunkToStderr(const QByteArray &chunk)
{
    if (chunk.isEmpty()) {
        return;
    }
    fwrite(chunk.constData(), 1, static_cast<size_t>(chunk.size()), stderr);
    fflush(stderr);
}

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

    if (const auto da = SteelCatalog::tryGetDoubleAngle2L(prof); da.has_value()) {
        const DoubleAngleSection &d = *da;
        *Aout = d.A_total_m2;
        const double i = std::max(d.i_buckling_m > 0.0 ? d.i_buckling_m : d.i_min_m, 1e-9);
        *Iout = std::max((*Aout) * i * i, 1e-14);
        return;
    }
    if (const auto ri = SteelCatalog::tryGetRolledI(prof); ri.has_value()) {
        *Aout = ri->A_m2;
        /** 2B düzlem: esnek burkulmaya muhafazakâr yaklaşım — min(Iy,Iz) ile rijitlik. */
        *Iout = std::max(std::min(ri->Iy_m4, ri->Iz_m4), 1e-14);
        return;
    }

    *Aout = A_nom;
    *Iout = I_truss_uniform;
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
/** Kolon veya gusset şeridi (kirış) — düğümde dönme rijitliği vardır; saf makas düğümlerinde rz serbest kalmamalı. */
bool hasIncidentColumnOrGussetBeam(const PortalFrameResult &r, int tag)
{
    for (const auto &m : r.members) {
        if (m.nodeI != tag && m.nodeJ != tag) {
            continue;
        }
        if (!m.isTruss) {
            return true;
        }
        if (m.trussRole == TrussMemberRole::GussetStrip) {
            return true;
        }
    }
    return false;
}

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
    const bool makasTekEksenelTruss =
        (in.trussMemberSectionForm >= 1 && in.trussMemberSectionForm <= 2);

    std::map<int, std::pair<double, double>> L;

    for (size_t mi = 0; mi < geo.members.size(); ++mi) {
        const auto &m = geo.members[mi];
        if (!m.carriesRoofLineLoad) {
            continue;
        }
        /** Mahya elastik kiriş: çatı yükü `eleLoad -beamUniform` ile verilir (düğüm yaması M’yi düşürürdü). */
        if (!m.isTruss && m.trussRole == TrussMemberRole::RafterBeam) {
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
        if (m.isTruss || m.trussRole != TrussMemberRole::Column) {
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

    if (makasTekEksenelTruss) {
        /** Hadde I makas: `truss` elemanı rz taşımaz — yalnızca makas/kolon-ayrı düğümlerde rz tutulu. */
        for (const auto &n : geo.nodes) {
            if (isBaseNode(geo, n.tag, W) || isGussetFootNode(geo, n.tag)) {
                continue;
            }
            if (!hasIncidentColumnOrGussetBeam(geo, n.tag)) {
                tcl += QStringLiteral("fix %1 0 0 1\n").arg(n.tag);
            }
        }
    }

    tcl += QStringLiteral("geomTransf Linear 1\n");

    for (const auto &m : geo.members) {
        const int eid = m.tag;
        if (!m.isTruss) {
            double Ab = Ac;
            double Ib = Ic;
            if (m.trussRole == TrussMemberRole::RafterBeam && trussPhysicalSizing) {
                if (const auto ri = SteelCatalog::tryGetRolledI(trussPhysicalSizing->rafterBeamProfile);
                    ri.has_value()) {
                    Ab = ri->A_m2;
                    Ib = std::max(ri->Iy_m4, 1e-14);
                }
            }
            tcl += QStringLiteral("element elasticBeamColumn %1 %2 %3 %4 %5 %6 1\n")
                       .arg(eid)
                       .arg(m.nodeI)
                       .arg(m.nodeJ)
                       .arg(Ab, 0, 'g', 17)
                       .arg(E, 0, 'g', 17)
                       .arg(Ib, 0, 'g', 17);
        } else if (makasTekEksenelTruss && m.trussRole != TrussMemberRole::GussetStrip) {
            double Atr = in.steelArea_m2;
            double ItrUnused = 0.0;
            trussElasticA_I(in, trussPhysicalSizing, m, &Atr, &ItrUnused);
            tcl += QStringLiteral("element truss %1 %2 %3 %4 1\n")
                       .arg(eid)
                       .arg(m.nodeI)
                       .arg(m.nodeJ)
                       .arg(Atr, 0, 'g', 17);
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
    for (const auto &m : geo.members) {
        if (m.isTruss || m.trussRole != TrussMemberRole::RafterBeam || !m.carriesRoofLineLoad) {
            continue;
        }
        double xi = 0.0;
        double yi = 0.0;
        double xj = 0.0;
        double yj = 0.0;
        if (!nodeCoords(geo, m.nodeI, xi, yi) || !nodeCoords(geo, m.nodeJ, xj, yj)) {
            continue;
        }
        const double wy = PortalSolver::rafterRoofUniformLocalY_N_per_m(w_N_m, xi, yi, xj, yj);
        if (std::abs(wy) < 1e-18) {
            continue;
        }
        tcl += QStringLiteral("  eleLoad -ele %1 -type -beamUniform %2\n")
                   .arg(m.tag)
                   .arg(wy, 0, 'g', 17);
    }
    tcl += QStringLiteral("}\n");

    QString nodeList;
    for (const auto &n : geo.nodes) {
        if (!nodeList.isEmpty()) {
            nodeList += ' ';
        }
        nodeList += QString::number(n.tag);
    }

    /** Kolon / mahya / gusset kirış: Tcl’de `elasticBeamColumn`; `basicForces` çıktısı eleman başına 3 sayı (N, M₁, M₂). */
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
        /** `basicForces`: elastik kiriş için doğrudan q = [N, M₁, M₂] (yerel temel sistem). `localForce` 6 bileşen
         *  P(0)…P(5) ile aynı M’leri üretir ancak eksenel bileşen farklı kombinasyonlarda moment okumasını zorlaştırır. */
        tcl += QStringLiteral("recorder Element -file {%1} -time -ele %2 basicForces\n")
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
            /** OpenSees `elasticBeamColumn` + `basicForces`: eleman başına 3 sayı (N, M₁, M₂) [N, N·m]. */
            if (idx + 2 < static_cast<int>(vals.size())) {
                m.axial_N = vals[idx];
                m.moment_i_Nm = vals[idx + 1];
                m.moment_j_Nm = vals[idx + 2];
            }
            idx += 3;
        }
    }
}

void parseTrussForcesLine(const QList<double> &vals, PortalFrameResult &geo, const PortalFrameInput &input)
{
    if (vals.size() < 2) {
        return;
    }
    const bool openSeesTrussBar =
        (input.trussMemberSectionForm >= 1 && input.trussMemberSectionForm <= 2);

    int idx = 1;
    for (auto &m : geo.members) {
        if (m.isTruss && m.trussRole != TrussMemberRole::GussetStrip) {
            if (openSeesTrussBar) {
                if (idx < static_cast<int>(vals.size())) {
                    m.axial_N = vals[idx];
                    m.moment_i_Nm = 0.0;
                    m.moment_j_Nm = 0.0;
                }
                idx += 1;
            } else {
                if (idx + 2 < static_cast<int>(vals.size())) {
                    m.axial_N = vals[idx];
                    m.moment_i_Nm = vals[idx + 1];
                    m.moment_j_Nm = vals[idx + 2];
                }
                idx += 3;
            }
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

bool expectsBeamForceRecorder(const PortalFrameResult &geo)
{
    for (const auto &m : geo.members) {
        if (!m.isTruss || m.trussRole == TrussMemberRole::GussetStrip) {
            return true;
        }
    }
    return false;
}

bool expectsTrussForceRecorder(const PortalFrameResult &geo)
{
    for (const auto &m : geo.members) {
        if (m.isTruss && m.trussRole != TrussMemberRole::GussetStrip) {
            return true;
        }
    }
    return false;
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

    const bool echo = openSeesConsoleEcho();
    QByteArray logOut;

    if (echo) {
        writeUtf8LineToStderr(QString());
        writeUtf8LineToStderr(
            QStringLiteral("── PortalSolver / OpenSees ──────────────────────────────────────"));
        writeUtf8LineToStderr(QStringLiteral("Geçici klasör: %1").arg(tmpDir.path()));
        writeUtf8LineToStderr(QStringLiteral("Tcl dosyası: %1").arg(tclPath));
        writeUtf8LineToStderr(QStringLiteral("OpenSees yürütülebilir: %1").arg(exe));
        writeUtf8LineToStderr(QStringLiteral("Çalışma dizini: %1").arg(tmpDir.path()));
        writeUtf8LineToStderr(
            QStringLiteral("Tasarım yükleri: qd,çatı = %1 kN/m, wd,kolon = %2 kN/m")
                .arg(q_roof_design_kN_m, 0, 'g', 12)
                .arg(w_column_design_kN_m, 0, 'g', 12));
        writeUtf8LineToStderr(QStringLiteral("OpenSees başlatılıyor… (çıktı canlı akar)"));
        writeUtf8LineToStderr(QStringLiteral("──────────────────────────────────────────────────────────────"));
    }

    proc.start();
    if (!proc.waitForStarted(30000)) {
        if (errorOut) {
            *errorOut = QStringLiteral("OpenSees başlatılamadı: %1").arg(proc.errorString());
        }
        return false;
    }

    QElapsedTimer wall;
    wall.start();
    constexpr int kOpenSeesTimeoutMs = 120000;
    while (proc.state() == QProcess::Running) {
        if (wall.elapsed() > kOpenSeesTimeoutMs) {
            proc.kill();
            proc.waitForFinished(5000);
            if (errorOut) {
                *errorOut = QStringLiteral(
                    "OpenSees zaman aşımı (120 s). "
                    "https://opensees.github.io/OpenSees/ üzerinden indirip OPENSEES_EXE ortam değişkenine tam yolu "
                    "yazın veya OpenSees.exe’yi PATH’e ekleyin.");
            }
            return false;
        }
        if (proc.waitForReadyRead(400)) {
            const QByteArray chunk = proc.readAllStandardOutput();
            logOut += chunk;
            if (echo) {
                writeUtf8ChunkToStderr(chunk);
            }
        } else if (proc.state() == QProcess::Running) {
            QThread::msleep(40);
        }
    }
    {
        const QByteArray tail = proc.readAllStandardOutput();
        logOut += tail;
        if (echo) {
            writeUtf8ChunkToStderr(tail);
        }
    }

    if (echo) {
        writeUtf8LineToStderr(QStringLiteral("──────────────────────────────────────────────────────────────"));
        writeUtf8LineToStderr(
            QStringLiteral("OpenSees süreç sonu: çıkış kodu %1, süre ~%2 ms")
                .arg(proc.exitCode())
                .arg(wall.elapsed()));
    }

    if (proc.exitCode() != 0) {
        if (errorOut) {
            *errorOut = QStringLiteral("OpenSees çıkış kodu %1:\n%2")
                            .arg(proc.exitCode())
                            .arg(QString::fromUtf8(logOut.left(4000)));
        }
        return false;
    }

    if (echo) {
        writeUtf8LineToStderr(
            QStringLiteral("OpenSees çıktı dosyaları okunuyor (düğüm yer değiştirmesi, kiriş ve makas kuvvetleri)…"));
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
    } else if (expectsBeamForceRecorder(ioResult)) {
        if (errorOut) {
            *errorOut = QStringLiteral("OpenSees kiriş/kolon kuvvet dosyası okunamadı veya boş: %1")
                            .arg(beamOut);
        }
        return false;
    }
    QList<double> trussVals;
    if (readLastNumberLine(trussOut, &trussVals)) {
        parseTrussForcesLine(trussVals, ioResult, input);
    } else if (expectsTrussForceRecorder(ioResult)) {
        if (errorOut) {
            *errorOut = QStringLiteral("OpenSees makas kuvvet dosyası okunamadı veya boş: %1")
                            .arg(trussOut);
        }
        return false;
    }

    QString lecSum;
    if (input.trussMemberSectionForm != 3
        && LectureTrussSolver::compareAxialVsOpenSees(input, ioResult, q_roof_design_kN_m, w_column_design_kN_m,
                                                      trussPhysicalSizing, &lecSum, nullptr)) {
        ioResult.lectureTrussComparisonNote = lecSum;
    } else if (input.trussMemberSectionForm != 3 && !lecSum.isEmpty()) {
        ioResult.lectureTrussComparisonNote = lecSum;
    }

    if (echo) {
        writeUtf8LineToStderr(QStringLiteral("OpenSees analiz adımı tamam."));
        writeUtf8LineToStderr(QString());
    }
    return true;
}
