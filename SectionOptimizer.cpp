#include "SectionOptimizer.h"

#include "PortalSolver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGammaM0 = 1.0;
constexpr double kGammaM1 = 1.0;
/** Makas çubuğu (2×L): ideal pimli düğümler — esnek burkulmada L_cr = K·L, K = 1,0 (çubuk boyu düğüm arası). */
constexpr double kTrussMemberBucklingK = 1.0;

double eulerNcr(double E_Pa, double Iz_m4, double K, double L_m)
{
    const double kl = K * L_m;
    return kPi * kPi * E_Pa * Iz_m4 / std::max(kl * kl, 1e-18);
}

/** TS EN 1993-1-1 6.3.1.2 — eğri b (α=0,34). */
double chiBuckling(double lambdaBar, double alpha)
{
    if (lambdaBar <= 1e-9) {
        return 1.0;
    }
    const double phi = 0.5 * (1.0 + alpha * (lambdaBar - 0.2) + lambdaBar * lambdaBar);
    const double r = phi * phi - lambdaBar * lambdaBar;
    const double den = phi + std::sqrt(std::max(r, 1e-18));
    return std::min(1.0 / den, 1.0);
}

double memberLength(const PortalFrameResult &r, int ti, int tj)
{
    double xi = 0.0;
    double yi = 0.0;
    double xj = 0.0;
    double yj = 0.0;
    bool fi = false;
    bool fj = false;
    for (const auto &n : r.nodes) {
        if (n.tag == ti) {
            xi = n.x;
            yi = n.y;
            fi = true;
        }
        if (n.tag == tj) {
            xj = n.x;
            yj = n.y;
            fj = true;
        }
    }
    if (!fi || !fj) {
        return 0.0;
    }
    return std::hypot(xj - xi, yj - yi);
}

/**
 * Portal kolon: TS EN 1993-1-1 elastik burkulma, eğri b.
 * - Etkili boy: K≈2 (çerçeve sallanması / makas bağlantısı — konservatif).
 * - Ncr = min(Ncr,y, Ncr,z); eksenel dayanım |N|/(χAfy) — OpenSees işaret karışırsa da güvenli.
 */
double columnUtilEC3(double N_N, double M_Nm, const RolledISection &sec, double fy_Pa, double E_Pa,
                     double columnHeight_m)
{
    const double A = sec.A_m2;
    const double Wy = sec.Wy_m3;
    const double Iy = sec.Iy_m4;
    const double Iz = sec.Iz_m4;
    if (A < 1e-18 || Wy < 1e-18 || Iy < 1e-18 || Iz < 1e-18) {
        return 1e9;
    }
    const double MRd = Wy * fy_Pa / kGammaM0;
    const double Nabs = std::abs(N_N);
    const double Mabs = std::abs(M_Nm);

    constexpr double K_portal = 2.0;
    const double Ncry = eulerNcr(E_Pa, Iy, K_portal, columnHeight_m);
    const double Ncrz = eulerNcr(E_Pa, Iz, K_portal, columnHeight_m);
    const double Ncr = std::min(Ncry, Ncrz);
    const double lambdaBar = std::sqrt(A * fy_Pa / std::max(Ncr, 1e-12));
    const double chi = chiBuckling(lambdaBar, 0.34);
    const double NRd = chi * A * fy_Pa / kGammaM1;
    return Nabs / std::max(NRd, 1e-9) + Mabs / std::max(MRd, 1e-9);
}

/**
 * Makas 2×L — TS EN 1993-1-1 6.3.1 eksenel esnek burkulma (eğri b, α = 0,34).
 * Çekme: N_pl,Rd = A·fy/γM0. Basınç: N_b,Rd = χ·A·fy/γM1; N_cr = π²EI/(KL)², I = A·i².
 * i: bileşik sırt sırta açı çifti (SteelCatalog). L: düğüm çalışma noktaları arası; K = 1 (pim–pim).
 */
double trussMemberUtilEC3(double N_N, double L_m, const DoubleAngleSection &da, double fy_Pa, double E_Pa,
                          double bucklingK = kTrussMemberBucklingK)
{
    const double A = da.A_total_m2;
    if (A < 1e-18 || L_m < 1e-9) {
        return 1e9;
    }
    const double i = std::max(da.i_buckling_m > 0.0 ? da.i_buckling_m : da.i_min_m, 1e-6);
    const double I = A * i * i;
    const double Npl = A * fy_Pa / kGammaM0;
    const double Nabs = std::abs(N_N);

    if (N_N >= 0.0) {
        return Nabs / std::max(Npl, 1e-9);
    }
    const double Ncr = eulerNcr(E_Pa, I, bucklingK, L_m);
    const double lambdaBar = std::sqrt(A * fy_Pa / std::max(Ncr, 1e-12));
    const double chi = chiBuckling(lambdaBar, 0.34);
    const double NRd = chi * A * fy_Pa / kGammaM1;
    return Nabs / std::max(NRd, 1e-9);
}

void approximateColumnForces(const PortalFrameInput &in, double qd_kN_per_m, double wd_kN_per_m, double *N_column_N,
                             double *M_base_Nm)
{
    const double W = in.spanWidth_m;
    const double Hc = in.columnHeight_m;
    const double Ha = PortalSolver::trussApexHeight_m(in);
    const double halfRafter = std::hypot(0.5 * W, Ha - Hc);
    const double N_kN = qd_kN_per_m * halfRafter;
    const double M_kNm = wd_kN_per_m * Hc * Hc / 2.0;
    *N_column_N = N_kN * 1000.0;
    *M_base_Nm = M_kNm * 1000.0;
}

double handTrussMaxAxialFromRoofLoad_N(const PortalFrameInput &in, double qd_kN_per_m)
{
    const double W = in.spanWidth_m;
    const double Ha = PortalSolver::trussApexHeight_m(in);
    const double Hc = in.columnHeight_m;
    const double Ls = std::hypot(0.5 * W, Ha - Hc);
    const double rise = std::max(Ha - Hc, 1e-6);
    const double q = qd_kN_per_m;
    const double M_kNm = q * Ls * Ls / 8.0;
    const double h_truss = std::max(0.22 * rise, 0.12 * Ls);
    const double N_kN = M_kNm / std::max(h_truss, 0.05);
    return N_kN * 1000.0 * 1.05;
}

bool hasUsableBeamForces(const PortalFrameResult &r)
{
    for (const auto &m : r.members) {
        if (m.isTruss) {
            continue;
        }
        if (std::abs(m.axial_N) > 1e-3 || std::abs(m.moment_i_Nm) > 1e-3 || std::abs(m.moment_j_Nm) > 1e-3) {
            return true;
        }
    }
    return false;
}

bool hasUsableTrussForces(const PortalFrameResult &r)
{
    for (const auto &m : r.members) {
        if (m.isTruss && std::abs(m.axial_N) > 1e-3) {
            return true;
        }
    }
    return false;
}

/** Üst hat 2×L zarfı: üst başlık + köşegen w=0,1 (üst hat ile aynı grup). */
static bool isChordTopSizingGroup(const MemberResult &m)
{
    if (!m.isTruss) {
        return false;
    }
    if (m.trussRole == TrussMemberRole::ChordTop) {
        return true;
    }
    return m.trussRole == TrussMemberRole::Web && m.webOptZone == 0;
}

static bool isWebZoneB(const MemberResult &m)
{
    return m.isTruss && m.trussRole == TrussMemberRole::Web && m.webOptZone == 1;
}

static bool isWebZoneC(const MemberResult &m)
{
    return m.isTruss && m.trussRole == TrussMemberRole::Web && m.webOptZone == 2;
}

static bool isWebZoneD(const MemberResult &m)
{
    return m.isTruss && m.trussRole == TrussMemberRole::Web && m.webOptZone == 3;
}

static bool isChordBottomMem(const MemberResult &m)
{
    return m.isTruss
           && (m.trussRole == TrussMemberRole::ChordBottom || m.trussRole == TrussMemberRole::GussetStrip);
}

static bool isEdgePostMem(const MemberResult &m)
{
    return m.isTruss && m.trussRole == TrussMemberRole::EdgePost;
}

static size_t doubleAngleCatalogIndex(const std::vector<DoubleAngleSection> &das, const QString &designation)
{
    for (size_t i = 0; i < das.size(); ++i) {
        if (das[i].designation == designation) {
            return i;
        }
    }
    return 0;
}

/** STR zarfı: verilen makas grubu için en hafif uygun 2×L (katalogda daBegin indeksinden itibaren). */
static void pick2LTrussGroupEnvelope(const std::vector<PortalFrameResult> &comboResults,
                                     const PortalFrameResult &refGeo,
                                     const std::vector<TurkishLoads::StrCombination> &combos,
                                     const std::vector<DoubleAngleSection> &das, double fy_Pa, double E,
                                     const std::function<bool(const MemberResult &)> &inGroup, QString *outProfile,
                                     double *outUtil, QString *outGovComb, double bucklingK = 1.0,
                                     size_t daBegin = 0)
{
    *outProfile = QStringLiteral("—");
    *outUtil = 0.0;
    outGovComb->clear();

    bool any = false;
    for (const auto &cr : comboResults) {
        for (const auto &m : cr.members) {
            if (inGroup(m)) {
                any = true;
                break;
            }
        }
        if (any) {
            break;
        }
    }
    if (!any) {
        return;
    }

    for (size_t di = daBegin; di < das.size(); ++di) {
        const auto &da = das[di];
        if (da.A_total_m2 < 1e-12) {
            continue;
        }
        double maxEnv = 0.0;
        QString govId;
        for (size_t ci = 0; ci < comboResults.size(); ++ci) {
            double comboMax = 0.0;
            for (const auto &m : comboResults[ci].members) {
                if (!inGroup(m)) {
                    continue;
                }
                const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
                comboMax = std::max(comboMax, trussMemberUtilEC3(m.axial_N, Lm, da, fy_Pa, E, bucklingK));
            }
            if (comboMax > maxEnv) {
                maxEnv = comboMax;
                govId = combos[ci].id;
            }
        }
        if (maxEnv <= 1.0 + 1e-6) {
            *outProfile = da.designation;
            *outUtil = maxEnv;
            *outGovComb = govId;
            return;
        }
    }
    if (!das.empty()) {
        const auto &last = das.back();
        double maxEnv = 0.0;
        QString govId;
        for (size_t ci = 0; ci < comboResults.size(); ++ci) {
            double comboMax = 0.0;
            for (const auto &m : comboResults[ci].members) {
                if (!inGroup(m)) {
                    continue;
                }
                const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
                comboMax = std::max(comboMax, trussMemberUtilEC3(m.axial_N, Lm, last, fy_Pa, E, bucklingK));
            }
            if (comboMax > maxEnv) {
                maxEnv = comboMax;
                govId = combos[ci].id;
            }
        }
        *outProfile = last.designation + QStringLiteral(" (aşım?)");
        *outUtil = maxEnv;
        *outGovComb = govId;
    }
}

/** Tek OpenSees sonucu ile bir makas grubu için 2×L seçimi. */
static void pick2LTrussGroupSingle(const PortalFrameResult &r, const PortalFrameResult &refGeo,
                                   const std::vector<DoubleAngleSection> &das, double fy_Pa, double E,
                                   const std::function<bool(const MemberResult &)> &inGroup, QString *outProfile,
                                   double *outUtil, double bucklingK = 1.0, size_t daBegin = 0)
{
    *outProfile = QStringLiteral("—");
    *outUtil = 0.0;

    bool any = false;
    for (const auto &m : r.members) {
        if (inGroup(m)) {
            any = true;
            break;
        }
    }
    if (!any) {
        return;
    }

    for (size_t di = daBegin; di < das.size(); ++di) {
        const auto &da = das[di];
        if (da.A_total_m2 < 1e-12) {
            continue;
        }
        double maxU = 0.0;
        for (const auto &m : r.members) {
            if (!inGroup(m)) {
                continue;
            }
            const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
            maxU = std::max(maxU, trussMemberUtilEC3(m.axial_N, Lm, da, fy_Pa, E, bucklingK));
        }
        if (maxU <= 1.0 + 1e-6) {
            *outProfile = da.designation;
            *outUtil = maxU;
            return;
        }
    }
    if (!das.empty()) {
        const auto &last = das.back();
        double maxU = 0.0;
        for (const auto &m : r.members) {
            if (!inGroup(m)) {
                continue;
            }
            const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
            maxU = std::max(maxU, trussMemberUtilEC3(m.axial_N, Lm, last, fy_Pa, E, bucklingK));
        }
        *outProfile = last.designation + QStringLiteral(" (aşım?)");
        *outUtil = maxU;
    }
}

} // namespace

SectionOptimizationResult optimizeSections(const PortalFrameInput &input, double qd_roof_kN_per_m,
                                           double wd_column_kN_per_m, double fy_MPa, ColumnFamily colFamily,
                                           const PortalFrameResult *openSeesResult);

SectionOptimizationResult optimizeSectionsEnvelope(const PortalFrameInput &input,
                                                   const std::vector<PortalFrameResult> &comboResults,
                                                   const std::vector<TurkishLoads::StrCombination> &combos,
                                                   double fy_MPa, ColumnFamily colFamily)
{
    SectionOptimizationResult r;
    if (comboResults.empty() || combos.empty() || comboResults.size() != combos.size()) {
        r.envelopeNote = QStringLiteral("Kombinasyon verisi eksik.");
        return r;
    }

    bool anyRealForces = false;
    for (const auto &cr : comboResults) {
        for (const auto &m : cr.members) {
            if (std::abs(m.axial_N) > 1.0 || std::abs(m.moment_i_Nm) > 1.0 || std::abs(m.moment_j_Nm) > 1.0) {
                anyRealForces = true;
                break;
            }
        }
        if (anyRealForces) {
            break;
        }
    }
    if (!anyRealForces) {
        double qm = 0.0;
        double wm = 0.0;
        for (const auto &c : combos) {
            qm = std::max(qm, c.q_roof_design_kN_m);
            wm = std::max(wm, c.w_column_design_kN_m);
        }
        SectionOptimizationResult hand = optimizeSections(input, qm, wm, fy_MPa, colFamily, nullptr);
        hand.envelopeNote =
            QStringLiteral("Uyarı: OpenSees eleman kuvvetleri anlamlı okunamadı; STR üst sınır q,w ile el boyutlama.");
        return hand;
    }

    const double fy_Pa = fy_MPa * 1e6;
    const double E = input.youngModulus_Pa;
    const double Hc = input.columnHeight_m;
    const PortalFrameResult &refGeo = comboResults.front();

    const auto &cols = SteelCatalog::sectionsForFamily(colFamily);
    r.columnProfile = QStringLiteral("—");
    r.columnUtilization = 0.0;

    for (const auto &sec : cols) {
        if (sec.A_m2 < 1e-12 || sec.Wy_m3 < 1e-12) {
            continue;
        }
        double maxEnv = 0.0;
        QString govId;
        for (size_t ci = 0; ci < comboResults.size(); ++ci) {
            double comboMax = 0.0;
            for (const auto &m : comboResults[ci].members) {
                if (m.isTruss) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                comboMax = std::max(comboMax, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, Hc));
            }
            if (comboMax > maxEnv) {
                maxEnv = comboMax;
                govId = combos[ci].id;
            }
        }
        if (maxEnv <= 1.0 + 1e-6) {
            r.columnProfile = sec.designation;
            r.columnUtilization = maxEnv;
            r.governingColumnCombinationId = govId;
            break;
        }
    }
    if (r.columnProfile == QStringLiteral("—") && !cols.empty()) {
        const auto &sec = cols.back();
        double maxEnv = 0.0;
        QString govId;
        for (size_t ci = 0; ci < comboResults.size(); ++ci) {
            double comboMax = 0.0;
            for (const auto &m : comboResults[ci].members) {
                if (m.isTruss) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                comboMax = std::max(comboMax, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, Hc));
            }
            if (comboMax > maxEnv) {
                maxEnv = comboMax;
                govId = combos[ci].id;
            }
        }
        r.columnProfile = sec.designation + QStringLiteral(" (aşım?)");
        r.columnUtilization = maxEnv;
        r.governingColumnCombinationId = govId;
    }

    double maxNdisp = 0.0;
    double maxMdisp = 0.0;
    for (size_t ci = 0; ci < comboResults.size(); ++ci) {
        for (const auto &m : comboResults[ci].members) {
            if (m.isTruss) {
                continue;
            }
            maxNdisp = std::max(maxNdisp, std::abs(m.axial_N));
            maxMdisp = std::max(maxMdisp, std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm)));
        }
    }
    r.approx_N_column_N = maxNdisp;
    r.approx_M_column_Nm = maxMdisp;

    const auto &das = SteelCatalog::doubleAngles2L();
    const size_t iSeedB = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));
    const size_t iSeedC = doubleAngleCatalogIndex(das, QStringLiteral("2xL 50x50x5"));
    const size_t iSeedD = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));

    r.trussTopChord2xL = QStringLiteral("—");
    r.trussBottomChord2xL = QStringLiteral("—");
    r.trussEdgePost2xL = QStringLiteral("—");
    r.trussWebB2xL = QStringLiteral("—");
    r.trussWebC2xL = QStringLiteral("—");
    r.trussWebD2xL = QStringLiteral("—");
    r.trussTopChordUtilization = 0.0;
    r.trussBottomChordUtilization = 0.0;
    r.trussEdgePostUtilization = 0.0;
    r.trussWebBUtilization = 0.0;
    r.trussWebCUtilization = 0.0;
    r.trussWebDUtilization = 0.0;

    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isChordTopSizingGroup, &r.trussTopChord2xL,
                             &r.trussTopChordUtilization, &r.governingTrussTopCombinationId);
    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isChordBottomMem, &r.trussBottomChord2xL,
                             &r.trussBottomChordUtilization, &r.governingTrussBottomCombinationId);
    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isEdgePostMem, &r.trussEdgePost2xL,
                             &r.trussEdgePostUtilization, &r.governingTrussEdgePostCombinationId);
    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isWebZoneB, &r.trussWebB2xL,
                             &r.trussWebBUtilization, &r.governingTrussWebBCombinationId, 1.0, iSeedB);
    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isWebZoneC, &r.trussWebC2xL,
                             &r.trussWebCUtilization, &r.governingTrussWebCCombinationId, 1.0, iSeedC);
    pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, isWebZoneD, &r.trussWebD2xL,
                             &r.trussWebDUtilization, &r.governingTrussWebDCombinationId, 1.0, iSeedD);

    r.trussUtilization = std::max({r.trussTopChordUtilization, r.trussBottomChordUtilization,
                                   r.trussEdgePostUtilization, r.trussWebBUtilization, r.trussWebCUtilization,
                                   r.trussWebDUtilization});
    r.trussProfile2xL = QStringLiteral("üst %1, alt %2, post %3, kşB %4, kşC %5, kşD %6")
                            .arg(r.trussTopChord2xL)
                            .arg(r.trussBottomChord2xL)
                            .arg(r.trussEdgePost2xL)
                            .arg(r.trussWebB2xL)
                            .arg(r.trussWebC2xL)
                            .arg(r.trussWebD2xL);
    r.governingTrussCombinationId =
        QStringLiteral("üst:%1 alt:%2 post:%3 kşB:%4 kşC:%5 kşD:%6")
            .arg(r.governingTrussTopCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussTopCombinationId)
            .arg(r.governingTrussBottomCombinationId.isEmpty() ? QStringLiteral("—")
                                                                 : r.governingTrussBottomCombinationId)
            .arg(r.governingTrussEdgePostCombinationId.isEmpty() ? QStringLiteral("—")
                                                                  : r.governingTrussEdgePostCombinationId)
            .arg(r.governingTrussWebBCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussWebBCombinationId)
            .arg(r.governingTrussWebCCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussWebCCombinationId)
            .arg(r.governingTrussWebDCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussWebDCombinationId);

    double maxTrussN = 0.0;
    for (const auto &cr : comboResults) {
        for (const auto &m : cr.members) {
            if (m.isTruss) {
                maxTrussN = std::max(maxTrussN, std::abs(m.axial_N));
            }
        }
    }
    r.approx_N_truss_max_N = maxTrussN;

    r.envelopeNote =
        QStringLiteral("TS 498 STR zarf: %1 kolon, %2 makas (TS EN 1993-1-1 basit χ, γM=1,0).")
            .arg(r.governingColumnCombinationId.isEmpty() ? QStringLiteral("—") : r.governingColumnCombinationId)
            .arg(r.governingTrussCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussCombinationId);

    return r;
}

SectionOptimizationResult optimizeSections(const PortalFrameInput &input, double qd_roof_kN_per_m,
                                         double wd_column_kN_per_m, double fy_MPa, ColumnFamily colFamily,
                                         const PortalFrameResult *openSeesResult)
{
    SectionOptimizationResult r;
    const double fy_Pa = fy_MPa * 1e6;
    const double E = input.youngModulus_Pa;
    const double Hc = input.columnHeight_m;

    const bool useOsTruss = openSeesResult && hasUsableTrussForces(*openSeesResult);
    const bool useOsColumn = openSeesResult && hasUsableBeamForces(*openSeesResult);

    PortalFrameResult geom;
    PortalSolver::buildPortalGeometry(input, geom);

    const auto &cols = SteelCatalog::sectionsForFamily(colFamily);
    r.columnProfile = QStringLiteral("—");
    r.columnUtilization = 0.0;

    if (useOsColumn) {
        for (const auto &sec : cols) {
            if (sec.A_m2 < 1e-12) {
                continue;
            }
            double maxUtil = 0.0;
            for (const auto &m : openSeesResult->members) {
                if (m.isTruss) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                maxUtil = std::max(maxUtil, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, Hc));
            }
            if (maxUtil <= 1.0 + 1e-6) {
                r.columnProfile = sec.designation;
                r.columnUtilization = maxUtil;
                break;
            }
        }
        if (r.columnProfile == QStringLiteral("—") && !cols.empty()) {
            const auto &sec = cols.back();
            double maxUtil = 0.0;
            for (const auto &m : openSeesResult->members) {
                if (m.isTruss) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                maxUtil = std::max(maxUtil, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, Hc));
            }
            r.columnProfile = sec.designation + QStringLiteral(" (aşım?)");
            r.columnUtilization = maxUtil;
        }
        for (const auto &m : openSeesResult->members) {
            if (!m.isTruss) {
                r.approx_N_column_N = std::max(r.approx_N_column_N, std::abs(m.axial_N));
                r.approx_M_column_Nm =
                    std::max(r.approx_M_column_Nm, std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm)));
            }
        }
    } else {
        double Nc = 0.0;
        double Mc = 0.0;
        approximateColumnForces(input, qd_roof_kN_per_m, wd_column_kN_per_m, &Nc, &Mc);
        r.approx_N_column_N = Nc;
        r.approx_M_column_Nm = Mc;
        for (const auto &sec : cols) {
            if (sec.A_m2 < 1e-12) {
                continue;
            }
            const double u = columnUtilEC3(-std::abs(Nc), Mc, sec, fy_Pa, E, Hc);
            if (u <= 1.0 + 1e-6) {
                r.columnProfile = sec.designation;
                r.columnUtilization = u;
                break;
            }
        }
        if (r.columnProfile == QStringLiteral("—") && !cols.empty()) {
            const auto &sec = cols.back();
            r.columnUtilization = columnUtilEC3(-std::abs(Nc), Mc, sec, fy_Pa, E, Hc);
            r.columnProfile = sec.designation + QStringLiteral(" (aşım?)");
        }
    }

    const auto &das = SteelCatalog::doubleAngles2L();
    const size_t iSeedB = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));
    const size_t iSeedC = doubleAngleCatalogIndex(das, QStringLiteral("2xL 50x50x5"));
    const size_t iSeedD = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));

    r.trussTopChord2xL = QStringLiteral("—");
    r.trussBottomChord2xL = QStringLiteral("—");
    r.trussEdgePost2xL = QStringLiteral("—");
    r.trussWebB2xL = QStringLiteral("—");
    r.trussWebC2xL = QStringLiteral("—");
    r.trussWebD2xL = QStringLiteral("—");
    r.trussTopChordUtilization = 0.0;
    r.trussBottomChordUtilization = 0.0;
    r.trussEdgePostUtilization = 0.0;
    r.trussWebBUtilization = 0.0;
    r.trussWebCUtilization = 0.0;
    r.trussWebDUtilization = 0.0;
    r.trussProfile2xL = QStringLiteral("—");
    r.trussUtilization = 0.0;

    double maxLTop = 1e-6;
    double maxLBot = 1e-6;
    double maxLEdge = 1e-6;
    double maxLB = 1e-6;
    double maxLC = 1e-6;
    double maxLD = 1e-6;
    for (const auto &m : geom.members) {
        if (!m.isTruss) {
            continue;
        }
        const double Lm = memberLength(geom, m.nodeI, m.nodeJ);
        if (isChordTopSizingGroup(m)) {
            maxLTop = std::max(maxLTop, Lm);
        } else if (isChordBottomMem(m)) {
            maxLBot = std::max(maxLBot, Lm);
        } else if (isEdgePostMem(m)) {
            maxLEdge = std::max(maxLEdge, Lm);
        } else if (isWebZoneB(m)) {
            maxLB = std::max(maxLB, Lm);
        } else if (isWebZoneC(m)) {
            maxLC = std::max(maxLC, Lm);
        } else if (isWebZoneD(m)) {
            maxLD = std::max(maxLD, Lm);
        }
    }

    if (useOsTruss) {
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isChordTopSizingGroup,
                               &r.trussTopChord2xL, &r.trussTopChordUtilization);
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isChordBottomMem,
                               &r.trussBottomChord2xL, &r.trussBottomChordUtilization);
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isEdgePostMem, &r.trussEdgePost2xL,
                               &r.trussEdgePostUtilization);
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isWebZoneB, &r.trussWebB2xL,
                               &r.trussWebBUtilization, 1.0, iSeedB);
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isWebZoneC, &r.trussWebC2xL,
                               &r.trussWebCUtilization, 1.0, iSeedC);
        pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, isWebZoneD, &r.trussWebD2xL,
                               &r.trussWebDUtilization, 1.0, iSeedD);
        for (const auto &m : openSeesResult->members) {
            if (m.isTruss) {
                r.approx_N_truss_max_N = std::max(r.approx_N_truss_max_N, std::abs(m.axial_N));
            }
        }
    } else {
        const double Nhand = handTrussMaxAxialFromRoofLoad_N(input, qd_roof_kN_per_m);
        r.approx_N_truss_max_N = Nhand;

        auto pickHand = [&](double Lbuck, QString *prof, double *util, double Kbuck, size_t daBegin) {
            *prof = QStringLiteral("—");
            *util = 0.0;
            for (size_t di = daBegin; di < das.size(); ++di) {
                const auto &da = das[di];
                if (da.A_total_m2 < 1e-12) {
                    continue;
                }
                const double u = trussMemberUtilEC3(-std::abs(Nhand), Lbuck, da, fy_Pa, E, Kbuck);
                if (u <= 1.0 + 1e-6) {
                    *prof = da.designation;
                    *util = u;
                    return;
                }
            }
            if (!das.empty()) {
                const auto &last = das.back();
                *util = trussMemberUtilEC3(-std::abs(Nhand), Lbuck, last, fy_Pa, E, Kbuck);
                *prof = last.designation + QStringLiteral(" (aşım?)");
            }
        };
        pickHand(maxLTop, &r.trussTopChord2xL, &r.trussTopChordUtilization, 1.0, 0);
        pickHand(maxLBot, &r.trussBottomChord2xL, &r.trussBottomChordUtilization, 1.0, 0);
        pickHand(maxLEdge, &r.trussEdgePost2xL, &r.trussEdgePostUtilization, 1.0, 0);
        pickHand(maxLB, &r.trussWebB2xL, &r.trussWebBUtilization, 1.0, iSeedB);
        pickHand(maxLC, &r.trussWebC2xL, &r.trussWebCUtilization, 1.0, iSeedC);
        pickHand(maxLD, &r.trussWebD2xL, &r.trussWebDUtilization, 1.0, iSeedD);
    }

    r.trussUtilization = std::max({r.trussTopChordUtilization, r.trussBottomChordUtilization,
                                   r.trussEdgePostUtilization, r.trussWebBUtilization, r.trussWebCUtilization,
                                   r.trussWebDUtilization});
    r.trussProfile2xL = QStringLiteral("üst %1, alt %2, post %3, kşB %4, kşC %5, kşD %6")
                            .arg(r.trussTopChord2xL)
                            .arg(r.trussBottomChord2xL)
                            .arg(r.trussEdgePost2xL)
                            .arg(r.trussWebB2xL)
                            .arg(r.trussWebC2xL)
                            .arg(r.trussWebD2xL);

    r.envelopeNote = QStringLiteral("Önizleme: STR üst sınır q,w ile el/tek senaryo (Calculate ile tam zarf).");

    return r;
}
