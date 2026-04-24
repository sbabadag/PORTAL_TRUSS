#include "SectionOptimizer.h"

#include "Buckling.h"
#include "OpenSeesRunner.h"
#include "PortalSolver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

#include <QStringList>

namespace {

constexpr double kGammaM0 = 1.0;
constexpr double kGammaM1 = 1.0;
/** Mahya (IPE): tek çubuk + el moment üst sınırı (wL²/8) basitleştirmesi için hafif güvenlik (süreklilik/şekil). */
constexpr double kRafterMomentSimplificationFactor = 1.12;
/** Makas çubuğu (2×L): ideal pimli düğümler — esnek burkulmada L_cr = K·L, K = 1,0 (çubuk boyu düğüm arası). */
constexpr double kTrussMemberBucklingK = 1.0;

double ltbCriticalMomentMcr_Nm(double E_Pa, double G_Pa, double Iz_m4, double It_m4, double Iw_m6, double Lb_m)
{
    // Doubly symmetric I, simplified Mcr (EC3 Annex F-style). Conservative: C1 = 1.0.
    if (E_Pa <= 0.0 || G_Pa <= 0.0 || Iz_m4 < 1e-24 || Lb_m < 1e-9) {
        return 0.0;
    }
    if (It_m4 <= 1e-28 && Iw_m6 <= 1e-35) {
        return 0.0;
    }
    constexpr double pi = 3.14159265358979323846;
    const double k = (pi * pi) * E_Pa * Iz_m4 / (Lb_m * Lb_m); // [N]
    const double a = std::max(Iw_m6, 0.0) / std::max(Iz_m4, 1e-24); // [m^2]
    const double b = (Lb_m * Lb_m) * G_Pa * std::max(It_m4, 0.0) / std::max((pi * pi) * E_Pa * Iz_m4, 1e-18); // [m^2]
    const double s = std::sqrt(std::max(a + b, 0.0)); // [m]
    return k * s; // [N·m]
}

double columnMcrMinFromBraceHeights_Nm(const std::vector<double> &braceFractions, double H, double E_Pa, double G_Pa,
                                      const RolledISection &sec)
{
    if (H < 1e-9) {
        return 0.0;
    }
    std::vector<double> pts;
    pts.reserve(braceFractions.size() + 2);
    pts.push_back(0.0);
    for (double f : braceFractions) {
        if (f > 1e-6 && f < 1.0 - 1e-6) {
            pts.push_back(f);
        }
    }
    pts.push_back(1.0);
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());

    double mcrMin = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double Lb = (pts[i + 1] - pts[i]) * H;
        if (Lb < 1e-6) {
            continue;
        }
        const double mcr = ltbCriticalMomentMcr_Nm(E_Pa, G_Pa, sec.Iz_m4, sec.It_m4, sec.Iw_m6, Lb);
        if (mcr <= 0.0) {
            continue;
        }
        mcrMin = (mcrMin <= 0.0) ? mcr : std::min(mcrMin, mcr);
    }
    if (mcrMin > 0.0) {
        return mcrMin;
    }
    return ltbCriticalMomentMcr_Nm(E_Pa, G_Pa, sec.Iz_m4, sec.It_m4, sec.Iw_m6, H);
}

/** Alt / üst başlık tutuluşları ayrı; her iki flanş da doluysa en küçük Mcr (muhafazakâr). */
double columnMcrLtbFromFlanges_Nm(const PortalFrameInput &in, const RolledISection &sec, double E_Pa)
{
    const double H = in.columnHeight_m;
    if (H < 1e-9) {
        return 0.0;
    }
    const double nu = 0.3;
    const double G = E_Pa / (2.0 * (1.0 + nu));
    const bool hasB = !in.columnLtbBottomFlangeBraceHeightFractions.empty();
    const bool hasT = !in.columnLtbTopFlangeBraceHeightFractions.empty();
    if (!hasB && !hasT) {
        return 0.0;
    }
    if (hasB && hasT) {
        const double mb =
            columnMcrMinFromBraceHeights_Nm(in.columnLtbBottomFlangeBraceHeightFractions, H, E_Pa, G, sec);
        const double mt =
            columnMcrMinFromBraceHeights_Nm(in.columnLtbTopFlangeBraceHeightFractions, H, E_Pa, G, sec);
        return std::min(mb, mt);
    }
    if (hasB) {
        return columnMcrMinFromBraceHeights_Nm(in.columnLtbBottomFlangeBraceHeightFractions, H, E_Pa, G, sec);
    }
    return columnMcrMinFromBraceHeights_Nm(in.columnLtbTopFlangeBraceHeightFractions, H, E_Pa, G, sec);
}

/** y-y burkulma: tek boy Ky·H (ara tutuluş listesi z-z için). */
static double ncrYColumnEuler(const PortalFrameInput &in, double E_Pa, double Iy_m4)
{
    const double H = in.columnHeight_m;
    if (H < 1e-12) {
        return std::numeric_limits<double>::max();
    }
    return Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iy_m4, in.columnBucklingKy, H);
}

/** z-z burkulma: `PortalSolver::columnElasticCriticalForceZz_N` ile aynı. */
static double ncrZColumnEuler(const PortalFrameInput &in, double E_Pa, double Iz_m4)
{
    return PortalSolver::columnElasticCriticalForceZz_N(E_Pa, Iz_m4, in);
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
 * Portal kolon: TS EN 1993-1-1 elastik burkulma + χ (giriş imperfection eğrisi).
 * - Ncr = min(Ncr,y, Ncr,z, Ncr,T); Ncr,T: çift simetrik I, It/Iw katalogdan (burulmalı–eğilme).
 * - Sınıf 4: EN 1993-1-5 etkin alan; λ̄ ve Nb,Rd’de Aeff; çekmede Npl brüt A.
 * - 2B çerçeve düzleminde eğilme: kolon web’i düzlemde → OpenSees M, y-y güçlü eksen (Wy); z-z (Wz) değil.
 */
double columnUtilEC3(double N_N, double M_Nm, const RolledISection &sec, double fy_Pa, double E_Pa,
                     const PortalFrameInput &in)
{
    const Ec3Buckling::ImperfectionCurve flexCurve =
        Ec3Buckling::imperfectionCurveFromOrdinal(in.columnBucklingCurveOrdinal);
    const Ec3Buckling::ImperfectionCurve ltbCurve =
        Ec3Buckling::imperfectionCurveFromOrdinal(in.columnLtbCurveOrdinal);
    const double A = sec.A_m2;
    const double Wy = sec.Wy_m3;
    const double Iy = sec.Iy_m4;
    const double Iz = sec.Iz_m4;
    if (A < 1e-18 || Wy < 1e-18 || Iy < 1e-18 || Iz < 1e-18) {
        return 1e9;
    }
    const double Nabs = std::abs(N_N);
    const double Mabs = std::abs(M_Nm);

    const double Ncry = ncrYColumnEuler(in, E_Pa, Iy);
    const double Ncrz = ncrZColumnEuler(in, E_Pa, Iz);
    double NcrT = std::numeric_limits<double>::infinity();
    if (sec.It_m4 > 1e-28 && sec.Iw_m6 > 1e-35 && E_Pa > 0.0) {
        const double nu = 0.3;
        const double G = E_Pa / (2.0 * (1.0 + nu));
        const double H = in.columnHeight_m;
        NcrT = Ec3Buckling::eulerTorsionalFlexuralCriticalForce_N(E_Pa, G, Iy, Iz, sec.It_m4, sec.Iw_m6,
                                                                  in.columnBucklingKz, H, A);
    }
    const double Ncr = std::min({Ncry, Ncrz, NcrT});

    double cf_mm = 0.0;
    double cw_mm = 0.0;
    bool geomOk = sec.h_mm > 1.0 && sec.b_mm > 1.0 && sec.tw_mm > 0.0 && sec.tf_mm > 0.0;
    if (geomOk) {
        const double rmm = (sec.r_mm > 0.5) ? sec.r_mm : std::max(1.5 * sec.tw_mm, 8.0);
        cf_mm = 0.5 * (sec.b_mm - sec.tw_mm - 2.0 * rmm);
        cw_mm = sec.h_mm - 2.0 * sec.tf_mm - 2.0 * rmm;
        geomOk = (cf_mm > 0.0 && cw_mm > 0.0);
    }
    double psiWeb = 1.0;
    if (geomOk) {
        const double s1 = N_N / A + M_Nm / Wy;
        const double s2 = N_N / A - M_Nm / Wy;
        const double sigMax = std::max(s1, s2);
        const double sigMin = std::min(s1, s2);
        if (sigMax > 1e-12) {
            psiWeb = std::clamp(sigMin / sigMax, -1.0, 1.0);
        }
    }
    Ec3Buckling::CrossSectionClass clsFl = Ec3Buckling::CrossSectionClass::Unknown;
    Ec3Buckling::CrossSectionClass clsW = Ec3Buckling::CrossSectionClass::Unknown;
    if (geomOk) {
        const double fyMPa = fy_Pa * 1e-6;
        clsFl = Ec3Buckling::classifyOutstandFlangeCompressionRolled(fyMPa, cf_mm, sec.tf_mm);
        clsW = Ec3Buckling::classifyInternalWebCompression(fyMPa, cw_mm, sec.tw_mm, psiWeb);
    }
    double Aeff = A;
    double WyEff = Wy;
    if (geomOk && clsFl != Ec3Buckling::CrossSectionClass::Unknown
        && clsW != Ec3Buckling::CrossSectionClass::Unknown
        && (clsFl == Ec3Buckling::CrossSectionClass::Class4 || clsW == Ec3Buckling::CrossSectionClass::Class4)) {
        Aeff = Ec3Buckling::effectiveAreaDoublySymmetricI_m2(A, fy_Pa * 1e-6, clsFl, clsW, cf_mm, sec.tf_mm, cw_mm,
                                                            sec.tw_mm, psiWeb);
        WyEff = Wy * (Aeff / A);
    }
    const double Achi = (Aeff < A - 1e-12 * A) ? Aeff : A;
    const double lambdaBar = Ec3Buckling::lambdaBarFlexuralCompression(Achi, fy_Pa, Ncr);
    const double chi = Ec3Buckling::chiFlexuralBuckling(lambdaBar, flexCurve);
    const double MRd = WyEff * fy_Pa / kGammaM0;
    double MRd_bend = MRd;
    {
        const double Mcr = columnMcrLtbFromFlanges_Nm(in, sec, E_Pa);
        if (Mcr > 1e-6 && MRd > 1e-12 && WyEff > 1e-18 && std::abs(M_Nm) > 1e-6) {
            const double lambdaLT = Ec3Buckling::lambdaBarLateralTorsional(WyEff, fy_Pa, Mcr);
            const double chiLT = Ec3Buckling::chiLateralTorsionalBuckling(lambdaLT, ltbCurve);
            MRd_bend = std::max(chiLT * MRd, 1e-9);
        }
    }
    /** OpenSees yerel eksen: N≥0 çekme, N<0 basınç. */
    const double NRd_axial = (N_N >= 0.0) ? Ec3Buckling::plasticAxialResistance_N(A, fy_Pa, kGammaM0)
                                         : Ec3Buckling::axialBucklingResistance_N(chi, Aeff, fy_Pa, kGammaM1);
    return Nabs / std::max(NRd_axial, 1e-9) + Mabs / std::max(MRd_bend, 1e-9);
}

/**
 * Makas 2×L — TS EN 1993-1-1 6.3.1 eksenel esnek burkulma (eğri b, α = 0,34).
 * Çekme: N_pl,Rd = A·fy/γM0. Basınç: N_b,Rd = χ·A·fy/γM1; N_cr = π²EI/(KL)², I = A·i².
 * i: bileşik sırt sırta açı çifti (SteelCatalog). L: düğüm çalışma noktaları arası; K = 1 (pim–pim).
 */
double trussMemberUtilEC3(double N_N, double L_m, const DoubleAngleSection &da, double fy_Pa, double E_Pa,
                          Ec3Buckling::ImperfectionCurve flexCurve, double bucklingK = kTrussMemberBucklingK)
{
    const double A = da.A_total_m2;
    if (A < 1e-18 || L_m < 1e-9) {
        return 1e9;
    }
    const double i = std::max(da.i_buckling_m > 0.0 ? da.i_buckling_m : da.i_min_m, 1e-6);
    const double I = A * i * i;
    const double Npl = Ec3Buckling::plasticAxialResistance_N(A, fy_Pa, kGammaM0);
    const double Nabs = std::abs(N_N);

    if (N_N >= 0.0) {
        return Nabs / std::max(Npl, 1e-9);
    }
    const double Ncr = Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, I, bucklingK, L_m);
    const double lambdaBar = Ec3Buckling::lambdaBarFlexuralCompression(A, fy_Pa, Ncr);
    const double chi = Ec3Buckling::chiFlexuralBuckling(lambdaBar, flexCurve);
    const double NRd = Ec3Buckling::axialBucklingResistance_N(chi, A, fy_Pa, kGammaM1);
    return Nabs / std::max(NRd, 1e-9);
}

/**
 * Makas tek hadde I (HEA/HEB/IPE — pütrel/çatı hattı tipi): TS EN 1993-1-1 6.3.1 eksenel.
 * Basınç: N_cr = min(N_cr,y, N_cr,z, N_cr,TF); L düğümler arası, K = 1 (pim–pim).
 */
double trussMemberUtilEC3RolledI(double N_N, double L_m, const RolledISection &sec, double fy_Pa, double E_Pa,
                                Ec3Buckling::ImperfectionCurve flexCurve, double bucklingK = kTrussMemberBucklingK)
{
    const double A = sec.A_m2;
    const double Iy = sec.Iy_m4;
    const double Iz = sec.Iz_m4;
    if (A < 1e-18 || L_m < 1e-9 || Iy < 1e-24 || Iz < 1e-24) {
        return 1e9;
    }
    const double Npl = Ec3Buckling::plasticAxialResistance_N(A, fy_Pa, kGammaM0);
    const double Nabs = std::abs(N_N);

    if (N_N >= 0.0) {
        return Nabs / std::max(Npl, 1e-9);
    }
    double Ncr = Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iy, bucklingK, L_m);
    Ncr = std::min(Ncr, Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iz, bucklingK, L_m));
    if (sec.It_m4 > 1e-28 && sec.Iw_m6 > 1e-35 && E_Pa > 0.0) {
        const double nu = 0.3;
        const double G = E_Pa / (2.0 * (1.0 + nu));
        const double NcrT = Ec3Buckling::eulerTorsionalFlexuralCriticalForce_N(E_Pa, G, Iy, Iz, sec.It_m4, sec.Iw_m6,
                                                                              bucklingK, L_m, A);
        if (NcrT > 0.0 && std::isfinite(NcrT)) {
            Ncr = std::min(Ncr, NcrT);
        }
    }
    const double lambdaBar = Ec3Buckling::lambdaBarFlexuralCompression(A, fy_Pa, Ncr);
    const double chi = Ec3Buckling::chiFlexuralBuckling(lambdaBar, flexCurve);
    const double NRd = Ec3Buckling::axialBucklingResistance_N(chi, A, fy_Pa, kGammaM1);
    return Nabs / std::max(NRd, 1e-9);
}

void approximateColumnForces(const PortalFrameInput &in, double qd_kN_per_m, double wd_kN_per_m, double *N_column_N,
                             double *M_base_Nm)
{
    const double W = in.spanWidth_m;
    const double Hc = in.columnHeight_m;
    /** qd is a vertical roof line load per horizontal metre; one column reaction is qd·W/2. */
    const double N_kN = qd_kN_per_m * 0.5 * W;
    /** Rüzgâr momenti (OpenSees kapalıyken): ankastre taban ≈ konsol wd·Hc²/2; mafsallı tabanda rz serbest — kabaca iki mesnetli kiriş wd·Hc²/8. */
    const double M_kNm = (in.columnBaseSupport == ColumnBaseSupport::Pinned) ? (wd_kN_per_m * Hc * Hc / 8.0)
                                                                           : (wd_kN_per_m * Hc * Hc / 2.0);
    *N_column_N = N_kN * 1000.0;
    *M_base_Nm = M_kNm * 1000.0;
}

double handTrussMaxAxialFromRoofLoad_N(const PortalFrameInput &in, double qd_kN_per_m)
{
    const double W = in.spanWidth_m;
    const double Ha = PortalSolver::trussApexHeight_m(in);
    const double Hc = in.columnHeight_m;
    const double rise = std::max(Ha - Hc, 1e-6);
    const double q = qd_kN_per_m;
    const double halfSpanHoriz = 0.5 * W;
    const double M_kNm = q * halfSpanHoriz * halfSpanHoriz / 8.0;
    /** Warren truss depth varies from W/40 at eave to W/11 at crown; use mid-depth for fallback chord force. */
    const double h_truss = std::max(0.5 * (W / 40.0 + W / 11.0), std::max(0.22 * rise, 0.05));
    const double N_kN = M_kNm / std::max(h_truss, 0.05);
    return N_kN * 1000.0 * 1.05;
}

bool hasUsableBeamForces(const PortalFrameResult &r)
{
    for (const auto &m : r.members) {
        if (m.trussRole != TrussMemberRole::Column) {
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

static bool isRafterBeamMem(const MemberResult &m)
{
    return !m.isTruss && m.trussRole == TrussMemberRole::RafterBeam;
}

static bool hasUsableRafterForces(const PortalFrameResult &r)
{
    for (const auto &m : r.members) {
        if (isRafterBeamMem(m)
            && (std::abs(m.axial_N) > 1.0 || std::abs(m.moment_i_Nm) > 1.0 || std::abs(m.moment_j_Nm) > 1.0)) {
            return true;
        }
    }
    return false;
}

static double maxRafterChordLength_m(const PortalFrameResult &refGeo)
{
    double Lm = 0.0;
    for (const auto &m : refGeo.members) {
        if (!isRafterBeamMem(m)) {
            continue;
        }
        Lm = std::max(Lm, memberLength(refGeo, m.nodeI, m.nodeJ));
    }
    return std::max(Lm, 1e-6);
}

static PortalFrameInput bucklingProxyForRafterChord(const PortalFrameInput &in, double chordLen_m)
{
    PortalFrameInput o = in;
    o.columnHeight_m = std::max(chordLen_m, 1e-6);
    o.columnBucklingKy = 1.0;
    o.columnBucklingKz = 1.0;
    o.columnLateralBraceHeightFractions.clear();
    o.columnZzBucklingBraceHeightFractions.clear();
    o.columnLateralBraceFromTrussGeometry = false;
    /**
     * `columnUtilEC3` LTB yalnızca alt/üst flanş listelerinden en az biri doluysa uygular; boşsa χ_LT=1 (tam M_Rd).
     * Mahyada ara yanal tutuluş yok kabulü: {0,1} değerleri `columnMcrMinFromBraceHeights` filtresinde elenir
     * (uçlar hariç), tek segment Lb = mahya boyu → Mcr ve χ_LT devreye girer.
     */
    o.columnLtbBottomFlangeBraceHeightFractions = {0.0, 1.0};
    o.columnLtbTopFlangeBraceHeightFractions.clear();
    return o;
}

static bool memberEndpoints_m(const PortalFrameResult &geo, int ni, int nj, double *xi, double *yi, double *xj,
                                double *yj)
{
    bool okI = false;
    bool okJ = false;
    for (const auto &n : geo.nodes) {
        if (n.tag == ni) {
            *xi = n.x;
            *yi = n.y;
            okI = true;
        }
        if (n.tag == nj) {
            *xj = n.x;
            *yj = n.y;
            okJ = true;
        }
    }
    return okI && okJ;
}

/** Uç momentleri + basit mesnetli yayılı yük üst sınırı (|M|_max ≥ w_y L²/8) — kayıtta yalnız uç M varken güvenli. */
static double rafterDesignMomentAxisNm(const MemberResult &m, const PortalFrameResult &geo,
                                       double q_roof_design_N_per_m)
{
    double xi = 0.0;
    double yi = 0.0;
    double xj = 0.0;
    double yj = 0.0;
    if (!memberEndpoints_m(geo, m.nodeI, m.nodeJ, &xi, &yi, &xj, &yj)) {
        return std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
    }
    const double L = std::hypot(xj - xi, yj - yi);
    const double wyAbs =
        std::abs(PortalSolver::rafterRoofUniformLocalY_N_per_m(q_roof_design_N_per_m, xi, yi, xj, yj));
    const double Mc_ss = kRafterMomentSimplificationFactor * wyAbs * L * L / 8.0;
    return std::max({std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm), Mc_ss});
}

/** Mahya: IPE yetersizse HEA, sonra HEB (aynı RolledISection + EC3 N+M). */
static const std::vector<RolledISection> *rafterMahyaCatalogOrder[] = {
    &SteelCatalog::ipeSections(),
    &SteelCatalog::sectionsForFamily(ColumnFamily::Hea),
    &SteelCatalog::sectionsForFamily(ColumnFamily::Heb),
};

static void rafterEnvelopeUtilForSection(const RolledISection &sec,
                                         const std::vector<PortalFrameResult> &comboResults,
                                         const PortalFrameResult &refGeo,
                                         const std::vector<TurkishLoads::StrCombination> &combos, double fy_Pa,
                                         double E, const PortalFrameInput &buckIn, double *outMaxEnv,
                                         QString *outGovComb)
{
    double maxEnv = 0.0;
    QString govId;
    for (size_t ci = 0; ci < comboResults.size(); ++ci) {
        const double qN = combos[ci].q_roof_design_kN_m * 1000.0;
        double comboMax = 0.0;
        for (const auto &m : comboResults[ci].members) {
            if (!isRafterBeamMem(m)) {
                continue;
            }
            const double Mx = rafterDesignMomentAxisNm(m, refGeo, qN);
            comboMax = std::max(comboMax, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, buckIn));
        }
        if (comboMax > maxEnv) {
            maxEnv = comboMax;
            govId = combos[ci].id;
        }
    }
    *outMaxEnv = maxEnv;
    *outGovComb = govId;
}

static void rafterSingleMaxUtilForSection(const RolledISection &sec, const PortalFrameResult &r,
                                          const PortalFrameResult &refGeo, double qN, double fy_Pa, double E,
                                          const PortalFrameInput &buckIn, double *outMaxU)
{
    double maxU = 0.0;
    for (const auto &m : r.members) {
        if (!isRafterBeamMem(m)) {
            continue;
        }
        const double Mx = rafterDesignMomentAxisNm(m, refGeo, qN);
        maxU = std::max(maxU, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, buckIn));
    }
    *outMaxU = maxU;
}

static void pickRafterIPEnvelope(const std::vector<PortalFrameResult> &comboResults,
                                  const PortalFrameResult &refGeo,
                                  const std::vector<TurkishLoads::StrCombination> &combos,
                                  const PortalFrameInput &input, double fy_Pa, double E, QString *outProfile,
                                  double *outUtil, QString *outGovComb)
{
    *outProfile = QStringLiteral("—");
    *outUtil = 0.0;
    outGovComb->clear();

    bool any = false;
    for (const auto &cr : comboResults) {
        for (const auto &m : cr.members) {
            if (isRafterBeamMem(m)) {
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

    const double Lch = maxRafterChordLength_m(refGeo);
    const PortalFrameInput buckIn = bucklingProxyForRafterChord(input, Lch);

    for (const std::vector<RolledISection> *cat : rafterMahyaCatalogOrder) {
        if (!cat || cat->empty()) {
            continue;
        }
        for (const auto &sec : *cat) {
            if (sec.A_m2 < 1e-12) {
                continue;
            }
            double maxEnv = 0.0;
            QString govId;
            rafterEnvelopeUtilForSection(sec, comboResults, refGeo, combos, fy_Pa, E, buckIn, &maxEnv, &govId);
            if (maxEnv <= 1.0 + 1e-6) {
                *outProfile = sec.designation;
                *outUtil = maxEnv;
                *outGovComb = govId;
                return;
            }
        }
    }
    const RolledISection *last = nullptr;
    const auto &heb = SteelCatalog::sectionsForFamily(ColumnFamily::Heb);
    const auto &hea = SteelCatalog::sectionsForFamily(ColumnFamily::Hea);
    const auto &ipe = SteelCatalog::ipeSections();
    if (!heb.empty()) {
        last = &heb.back();
    } else if (!hea.empty()) {
        last = &hea.back();
    } else if (!ipe.empty()) {
        last = &ipe.back();
    }
    if (last != nullptr) {
        double maxEnv = 0.0;
        QString govId;
        rafterEnvelopeUtilForSection(*last, comboResults, refGeo, combos, fy_Pa, E, buckIn, &maxEnv, &govId);
        *outProfile = last->designation + QStringLiteral(" (aşım?)");
        *outUtil = maxEnv;
        *outGovComb = govId;
    }
}

static void pickRafterIPESingle(const PortalFrameResult &r, const PortalFrameResult &refGeo,
                                const PortalFrameInput &input, double q_roof_design_kN_per_m, double fy_Pa, double E,
                                QString *outProfile, double *outUtil)
{
    *outProfile = QStringLiteral("—");
    *outUtil = 0.0;

    bool any = false;
    for (const auto &m : r.members) {
        if (isRafterBeamMem(m)) {
            any = true;
            break;
        }
    }
    if (!any) {
        return;
    }

    const double qN = q_roof_design_kN_per_m * 1000.0;
    const double Lch = maxRafterChordLength_m(refGeo);
    const PortalFrameInput buckIn = bucklingProxyForRafterChord(input, Lch);

    for (const std::vector<RolledISection> *cat : rafterMahyaCatalogOrder) {
        if (!cat || cat->empty()) {
            continue;
        }
        for (const auto &sec : *cat) {
            if (sec.A_m2 < 1e-12) {
                continue;
            }
            double maxU = 0.0;
            rafterSingleMaxUtilForSection(sec, r, refGeo, qN, fy_Pa, E, buckIn, &maxU);
            if (maxU <= 1.0 + 1e-6) {
                *outProfile = sec.designation;
                *outUtil = maxU;
                return;
            }
        }
    }
    const RolledISection *last = nullptr;
    const auto &heb = SteelCatalog::sectionsForFamily(ColumnFamily::Heb);
    const auto &hea = SteelCatalog::sectionsForFamily(ColumnFamily::Hea);
    const auto &ipe = SteelCatalog::ipeSections();
    if (!heb.empty()) {
        last = &heb.back();
    } else if (!hea.empty()) {
        last = &hea.back();
    } else if (!ipe.empty()) {
        last = &ipe.back();
    }
    if (last != nullptr) {
        double maxU = 0.0;
        rafterSingleMaxUtilForSection(*last, r, refGeo, qN, fy_Pa, E, buckIn, &maxU);
        *outProfile = last->designation + QStringLiteral(" (aşım?)");
        *outUtil = maxU;
    }
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
                                     Ec3Buckling::ImperfectionCurve flexCurve,
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
                comboMax =
                    std::max(comboMax, trussMemberUtilEC3(m.axial_N, Lm, da, fy_Pa, E, flexCurve, bucklingK));
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
                comboMax =
                    std::max(comboMax, trussMemberUtilEC3(m.axial_N, Lm, last, fy_Pa, E, flexCurve, bucklingK));
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
                                   Ec3Buckling::ImperfectionCurve flexCurve,
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
            maxU = std::max(maxU, trussMemberUtilEC3(m.axial_N, Lm, da, fy_Pa, E, flexCurve, bucklingK));
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
            maxU = std::max(maxU, trussMemberUtilEC3(m.axial_N, Lm, last, fy_Pa, E, flexCurve, bucklingK));
        }
        *outProfile = last.designation + QStringLiteral(" (aşım?)");
        *outUtil = maxU;
    }
}

/** STR zarfı: makas grubu için en hafif uygun hadde I (katalog sırası: kütle artan). */
static void pickRolledITrussGroupEnvelope(const std::vector<PortalFrameResult> &comboResults,
                                          const PortalFrameResult &refGeo,
                                          const std::vector<TurkishLoads::StrCombination> &combos,
                                          const std::vector<RolledISection> &rolled, double fy_Pa, double E,
                                          Ec3Buckling::ImperfectionCurve flexCurve,
                                          const std::function<bool(const MemberResult &)> &inGroup, QString *outProfile,
                                          double *outUtil, QString *outGovComb, double bucklingK = 1.0,
                                          size_t rolledBegin = 0)
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

    for (size_t si = rolledBegin; si < rolled.size(); ++si) {
        const auto &sec = rolled[si];
        if (sec.A_m2 < 1e-12) {
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
                comboMax =
                    std::max(comboMax, trussMemberUtilEC3RolledI(m.axial_N, Lm, sec, fy_Pa, E, flexCurve, bucklingK));
            }
            if (comboMax > maxEnv) {
                maxEnv = comboMax;
                govId = combos[ci].id;
            }
        }
        if (maxEnv <= 1.0 + 1e-6) {
            *outProfile = sec.designation;
            *outUtil = maxEnv;
            *outGovComb = govId;
            return;
        }
    }
    if (!rolled.empty()) {
        const auto &last = rolled.back();
        double maxEnv = 0.0;
        QString govId;
        for (size_t ci = 0; ci < comboResults.size(); ++ci) {
            double comboMax = 0.0;
            for (const auto &m : comboResults[ci].members) {
                if (!inGroup(m)) {
                    continue;
                }
                const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
                comboMax = std::max(comboMax,
                                    trussMemberUtilEC3RolledI(m.axial_N, Lm, last, fy_Pa, E, flexCurve, bucklingK));
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

static void pickRolledITrussGroupSingle(const PortalFrameResult &r, const PortalFrameResult &refGeo,
                                        const std::vector<RolledISection> &rolled, double fy_Pa, double E,
                                        Ec3Buckling::ImperfectionCurve flexCurve,
                                        const std::function<bool(const MemberResult &)> &inGroup, QString *outProfile,
                                        double *outUtil, double bucklingK = 1.0, size_t rolledBegin = 0)
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

    for (size_t si = rolledBegin; si < rolled.size(); ++si) {
        const auto &sec = rolled[si];
        if (sec.A_m2 < 1e-12) {
            continue;
        }
        double maxU = 0.0;
        for (const auto &m : r.members) {
            if (!inGroup(m)) {
                continue;
            }
            const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
            maxU = std::max(maxU, trussMemberUtilEC3RolledI(m.axial_N, Lm, sec, fy_Pa, E, flexCurve, bucklingK));
        }
        if (maxU <= 1.0 + 1e-6) {
            *outProfile = sec.designation;
            *outUtil = maxU;
            return;
        }
    }
    if (!rolled.empty()) {
        const auto &last = rolled.back();
        double maxU = 0.0;
        for (const auto &m : r.members) {
            if (!inGroup(m)) {
                continue;
            }
            const double Lm = memberLength(refGeo, m.nodeI, m.nodeJ);
            maxU = std::max(maxU, trussMemberUtilEC3RolledI(m.axial_N, Lm, last, fy_Pa, E, flexCurve, bucklingK));
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
                if (m.trussRole != TrussMemberRole::Column) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                comboMax = std::max(comboMax, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, input));
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
                if (m.trussRole != TrussMemberRole::Column) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                comboMax = std::max(comboMax, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, input));
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
            if (m.trussRole != TrussMemberRole::Column) {
                continue;
            }
            maxNdisp = std::max(maxNdisp, std::abs(m.axial_N));
            maxMdisp = std::max(maxMdisp, std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm)));
        }
    }
    r.approx_N_column_N = maxNdisp;
    r.approx_M_column_Nm = maxMdisp;

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

    const Ec3Buckling::ImperfectionCurve trussCurve =
        Ec3Buckling::imperfectionCurveFromOrdinal(input.trussBucklingCurveOrdinal);

    const bool formIpeMahya = (input.trussMemberSectionForm == 3);
    const bool trussRolledI = (input.trussMemberSectionForm == 1 || input.trussMemberSectionForm == 2);

    if (formIpeMahya) {
        pickRafterIPEnvelope(comboResults, refGeo, combos, input, fy_Pa, E, &r.rafterBeamProfile,
                              &r.rafterBeamUtilization, &r.governingRafterCombinationId);
    } else if (!trussRolledI) {
        const auto &das = SteelCatalog::doubleAngles2L();
        const size_t iSeedB = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));
        const size_t iSeedC = doubleAngleCatalogIndex(das, QStringLiteral("2xL 50x50x5"));
        const size_t iSeedD = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));

        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isChordTopSizingGroup,
                                 &r.trussTopChord2xL, &r.trussTopChordUtilization, &r.governingTrussTopCombinationId);
        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isChordBottomMem,
                                 &r.trussBottomChord2xL, &r.trussBottomChordUtilization,
                                 &r.governingTrussBottomCombinationId);
        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isEdgePostMem,
                                 &r.trussEdgePost2xL, &r.trussEdgePostUtilization, &r.governingTrussEdgePostCombinationId);
        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isWebZoneB, &r.trussWebB2xL,
                                 &r.trussWebBUtilization, &r.governingTrussWebBCombinationId, 1.0, iSeedB);
        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isWebZoneC, &r.trussWebC2xL,
                                 &r.trussWebCUtilization, &r.governingTrussWebCCombinationId, 1.0, iSeedC);
        pick2LTrussGroupEnvelope(comboResults, refGeo, combos, das, fy_Pa, E, trussCurve, isWebZoneD, &r.trussWebD2xL,
                                 &r.trussWebDUtilization, &r.governingTrussWebDCombinationId, 1.0, iSeedD);
    } else {
        const ColumnFamily trussFam = static_cast<ColumnFamily>(input.trussMemberSectionForm - 1);
        const auto &rolled = SteelCatalog::sectionsForFamily(trussFam);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isChordTopSizingGroup,
                                      &r.trussTopChord2xL, &r.trussTopChordUtilization, &r.governingTrussTopCombinationId);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isChordBottomMem,
                                      &r.trussBottomChord2xL, &r.trussBottomChordUtilization,
                                      &r.governingTrussBottomCombinationId);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isEdgePostMem,
                                      &r.trussEdgePost2xL, &r.trussEdgePostUtilization,
                                      &r.governingTrussEdgePostCombinationId);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isWebZoneB,
                                      &r.trussWebB2xL, &r.trussWebBUtilization, &r.governingTrussWebBCombinationId);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isWebZoneC,
                                      &r.trussWebC2xL, &r.trussWebCUtilization, &r.governingTrussWebCCombinationId);
        pickRolledITrussGroupEnvelope(comboResults, refGeo, combos, rolled, fy_Pa, E, trussCurve, isWebZoneD,
                                      &r.trussWebD2xL, &r.trussWebDUtilization, &r.governingTrussWebDCombinationId);
    }

    if (formIpeMahya) {
        r.trussUtilization = r.rafterBeamUtilization;
        r.trussProfile2xL = QStringLiteral("mahya %1").arg(r.rafterBeamProfile);
        r.governingTrussCombinationId =
            r.governingRafterCombinationId.isEmpty() ? QStringLiteral("—") : r.governingRafterCombinationId;
    } else {
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
                .arg(r.governingTrussWebBCombinationId.isEmpty() ? QStringLiteral("—")
                                                                   : r.governingTrussWebBCombinationId)
                .arg(r.governingTrussWebCCombinationId.isEmpty() ? QStringLiteral("—")
                                                                   : r.governingTrussWebCCombinationId)
                .arg(r.governingTrussWebDCombinationId.isEmpty() ? QStringLiteral("—")
                                                                 : r.governingTrussWebDCombinationId);
    }

    double maxTrussN = 0.0;
    for (const auto &cr : comboResults) {
        for (const auto &m : cr.members) {
            if (m.isTruss || isRafterBeamMem(m)) {
                maxTrussN = std::max(maxTrussN, std::abs(m.axial_N));
            }
        }
    }
    r.approx_N_truss_max_N = maxTrussN;

    QString trussMode;
    if (formIpeMahya) {
        trussMode = QStringLiteral("Mahya IPE→HEA→HEB (kirış N+M)");
    } else if (trussRolledI) {
        trussMode = QStringLiteral("hadde I (Eksenel χ, Ncr=min y-y,z-z,TF)");
    } else {
        trussMode = QStringLiteral("2×L açı");
    }
    r.envelopeNote =
        QStringLiteral("TS 498 STR zarf: %1 kolon, %2 makas — %3 (γM=1,0).")
            .arg(r.governingColumnCombinationId.isEmpty() ? QStringLiteral("—") : r.governingColumnCombinationId)
            .arg(r.governingTrussCombinationId.isEmpty() ? QStringLiteral("—") : r.governingTrussCombinationId)
            .arg(trussMode);
    if (input.columnLtbBottomFlangeBraceHeightFractions.empty()
        && input.columnLtbTopFlangeBraceHeightFractions.empty()) {
        r.envelopeNote += QStringLiteral(" Kolon LTB: flanş tutuluşu girilmediği için uygulanmadı.");
    }

    return r;
}

SectionOptimizationResult optimizeSections(const PortalFrameInput &input, double qd_roof_kN_per_m,
                                         double wd_column_kN_per_m, double fy_MPa, ColumnFamily colFamily,
                                         const PortalFrameResult *openSeesResult)
{
    SectionOptimizationResult r;
    const double fy_Pa = fy_MPa * 1e6;
    const double E = input.youngModulus_Pa;
    const Ec3Buckling::ImperfectionCurve trussCurve =
        Ec3Buckling::imperfectionCurveFromOrdinal(input.trussBucklingCurveOrdinal);

    const bool useOsTruss = openSeesResult && hasUsableTrussForces(*openSeesResult);
    const bool useOsRafter = openSeesResult && hasUsableRafterForces(*openSeesResult);
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
                if (m.trussRole != TrussMemberRole::Column) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                maxUtil = std::max(maxUtil, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, input));
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
                if (m.trussRole != TrussMemberRole::Column) {
                    continue;
                }
                const double Mx = std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm));
                maxUtil = std::max(maxUtil, columnUtilEC3(m.axial_N, Mx, sec, fy_Pa, E, input));
            }
            r.columnProfile = sec.designation + QStringLiteral(" (aşım?)");
            r.columnUtilization = maxUtil;
        }
        for (const auto &m : openSeesResult->members) {
            if (m.trussRole == TrussMemberRole::Column) {
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
            const double u = columnUtilEC3(-std::abs(Nc), Mc, sec, fy_Pa, E, input);
            if (u <= 1.0 + 1e-6) {
                r.columnProfile = sec.designation;
                r.columnUtilization = u;
                break;
            }
        }
        if (r.columnProfile == QStringLiteral("—") && !cols.empty()) {
            const auto &sec = cols.back();
            r.columnUtilization = columnUtilEC3(-std::abs(Nc), Mc, sec, fy_Pa, E, input);
            r.columnProfile = sec.designation + QStringLiteral(" (aşım?)");
        }
    }

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

    const bool formIpeMahya = (input.trussMemberSectionForm == 3);
    const bool trussRolledI = (input.trussMemberSectionForm == 1 || input.trussMemberSectionForm == 2);

    double maxLTop = 1e-6;
    double maxLBot = 1e-6;
    double maxLEdge = 1e-6;
    double maxLB = 1e-6;
    double maxLC = 1e-6;
    double maxLD = 1e-6;
    if (!formIpeMahya) {
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
    }

    if (formIpeMahya) {
        if (useOsRafter) {
            pickRafterIPESingle(*openSeesResult, *openSeesResult, input, qd_roof_kN_per_m, fy_Pa, E,
                                &r.rafterBeamProfile, &r.rafterBeamUtilization);
            for (const auto &m : openSeesResult->members) {
                if (isRafterBeamMem(m)) {
                    r.approx_N_truss_max_N = std::max(r.approx_N_truss_max_N, std::abs(m.axial_N));
                }
            }
        } else {
            const double Lsl = maxRafterChordLength_m(geom);
            const PortalFrameInput buckIn = bucklingProxyForRafterChord(input, Lsl);
            const double W = input.spanWidth_m;
            const double wAlong = qd_roof_kN_per_m * 1000.0 * (0.5 * W) / std::max(Lsl, 1e-9);
            const double Mc_hand = kRafterMomentSimplificationFactor * wAlong * Lsl * Lsl / 8.0;
            r.approx_N_truss_max_N = 0.0;
            bool rafterHandOk = false;
            for (const std::vector<RolledISection> *cat : rafterMahyaCatalogOrder) {
                if (!cat || cat->empty()) {
                    continue;
                }
                for (const auto &sec : *cat) {
                    if (sec.A_m2 < 1e-12) {
                        continue;
                    }
                    const double u = columnUtilEC3(0.0, Mc_hand, sec, fy_Pa, E, buckIn);
                    if (u <= 1.0 + 1e-6) {
                        r.rafterBeamProfile = sec.designation;
                        r.rafterBeamUtilization = u;
                        rafterHandOk = true;
                        break;
                    }
                }
                if (rafterHandOk) {
                    break;
                }
            }
            if (!rafterHandOk) {
                const auto &heb = SteelCatalog::sectionsForFamily(ColumnFamily::Heb);
                const auto &hea = SteelCatalog::sectionsForFamily(ColumnFamily::Hea);
                const auto &ipe = SteelCatalog::ipeSections();
                const RolledISection *last = nullptr;
                if (!heb.empty()) {
                    last = &heb.back();
                } else if (!hea.empty()) {
                    last = &hea.back();
                } else if (!ipe.empty()) {
                    last = &ipe.back();
                }
                if (last != nullptr) {
                    r.rafterBeamUtilization = columnUtilEC3(0.0, Mc_hand, *last, fy_Pa, E, buckIn);
                    r.rafterBeamProfile = last->designation + QStringLiteral(" (aşım?)");
                }
            }
        }
    } else if (!trussRolledI) {
        const auto &das = SteelCatalog::doubleAngles2L();
        const size_t iSeedB = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));
        const size_t iSeedC = doubleAngleCatalogIndex(das, QStringLiteral("2xL 50x50x5"));
        const size_t iSeedD = doubleAngleCatalogIndex(das, QStringLiteral("2xL 60x60x6"));

        if (useOsTruss) {
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isChordTopSizingGroup,
                                   &r.trussTopChord2xL, &r.trussTopChordUtilization);
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isChordBottomMem,
                                   &r.trussBottomChord2xL, &r.trussBottomChordUtilization);
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isEdgePostMem,
                                   &r.trussEdgePost2xL, &r.trussEdgePostUtilization);
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isWebZoneB,
                                   &r.trussWebB2xL, &r.trussWebBUtilization, 1.0, iSeedB);
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isWebZoneC,
                                   &r.trussWebC2xL, &r.trussWebCUtilization, 1.0, iSeedC);
            pick2LTrussGroupSingle(*openSeesResult, *openSeesResult, das, fy_Pa, E, trussCurve, isWebZoneD,
                                   &r.trussWebD2xL, &r.trussWebDUtilization, 1.0, iSeedD);
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
                    const double u = trussMemberUtilEC3(-std::abs(Nhand), Lbuck, da, fy_Pa, E, trussCurve, Kbuck);
                    if (u <= 1.0 + 1e-6) {
                        *prof = da.designation;
                        *util = u;
                        return;
                    }
                }
                if (!das.empty()) {
                    const auto &last = das.back();
                    *util = trussMemberUtilEC3(-std::abs(Nhand), Lbuck, last, fy_Pa, E, trussCurve, Kbuck);
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
    } else {
        const auto &rolled =
            SteelCatalog::sectionsForFamily(static_cast<ColumnFamily>(input.trussMemberSectionForm - 1));
        if (useOsTruss) {
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve,
                                        isChordTopSizingGroup, &r.trussTopChord2xL, &r.trussTopChordUtilization);
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve,
                                        isChordBottomMem, &r.trussBottomChord2xL, &r.trussBottomChordUtilization);
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve, isEdgePostMem,
                                        &r.trussEdgePost2xL, &r.trussEdgePostUtilization);
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve, isWebZoneB,
                                        &r.trussWebB2xL, &r.trussWebBUtilization);
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve, isWebZoneC,
                                        &r.trussWebC2xL, &r.trussWebCUtilization);
            pickRolledITrussGroupSingle(*openSeesResult, *openSeesResult, rolled, fy_Pa, E, trussCurve, isWebZoneD,
                                        &r.trussWebD2xL, &r.trussWebDUtilization);
            for (const auto &m : openSeesResult->members) {
                if (m.isTruss) {
                    r.approx_N_truss_max_N = std::max(r.approx_N_truss_max_N, std::abs(m.axial_N));
                }
            }
        } else {
            const double Nhand = handTrussMaxAxialFromRoofLoad_N(input, qd_roof_kN_per_m);
            r.approx_N_truss_max_N = Nhand;

            auto pickHandRolled = [&](double Lbuck, QString *prof, double *util, double Kbuck, size_t beginIdx) {
                *prof = QStringLiteral("—");
                *util = 0.0;
                for (size_t si = beginIdx; si < rolled.size(); ++si) {
                    const auto &sec = rolled[si];
                    if (sec.A_m2 < 1e-12) {
                        continue;
                    }
                    const double u =
                        trussMemberUtilEC3RolledI(-std::abs(Nhand), Lbuck, sec, fy_Pa, E, trussCurve, Kbuck);
                    if (u <= 1.0 + 1e-6) {
                        *prof = sec.designation;
                        *util = u;
                        return;
                    }
                }
                if (!rolled.empty()) {
                    const auto &last = rolled.back();
                    *util = trussMemberUtilEC3RolledI(-std::abs(Nhand), Lbuck, last, fy_Pa, E, trussCurve, Kbuck);
                    *prof = last.designation + QStringLiteral(" (aşım?)");
                }
            };
            pickHandRolled(maxLTop, &r.trussTopChord2xL, &r.trussTopChordUtilization, 1.0, 0);
            pickHandRolled(maxLBot, &r.trussBottomChord2xL, &r.trussBottomChordUtilization, 1.0, 0);
            pickHandRolled(maxLEdge, &r.trussEdgePost2xL, &r.trussEdgePostUtilization, 1.0, 0);
            pickHandRolled(maxLB, &r.trussWebB2xL, &r.trussWebBUtilization, 1.0, 0);
            pickHandRolled(maxLC, &r.trussWebC2xL, &r.trussWebCUtilization, 1.0, 0);
            pickHandRolled(maxLD, &r.trussWebD2xL, &r.trussWebDUtilization, 1.0, 0);
        }
    }

    if (formIpeMahya) {
        r.trussUtilization = r.rafterBeamUtilization;
        r.trussProfile2xL = QStringLiteral("mahya %1").arg(r.rafterBeamProfile);
    } else {
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
    }

    r.envelopeNote = QStringLiteral("Önizleme: STR üst sınır q,w ile el/tek senaryo (Calculate ile tam zarf).");

    return r;
}

QString runPortalSelfCheckDefaultInput()
{
    const double epsRel = 1e-9;
    const double epsAbs = 1e-6;
    PortalFrameInput in;
    QString err;

    if (!PortalSolver().validate(in, &err)) {
        return QStringLiteral("FAIL: validate — %1").arg(err);
    }

    const double W = in.spanWidth_m;
    const double Hc = in.columnHeight_m;
    const double S = in.trussAxisSpacingY_m;
    const auto ch = in.lineLoadsCharacteristic();

    const double wl_exp = in.wl_kN_per_m2 * S;
    if (std::abs(ch.wl - wl_exp) > epsAbs + epsRel * std::max(std::abs(wl_exp), 1.0)) {
        return QStringLiteral("FAIL: characteristic wl line (expected %1, got %2)").arg(wl_exp).arg(ch.wl);
    }
    const double sn_exp = in.sk_kN_per_m2 * 0.8 * S;
    if (std::abs(ch.sn - sn_exp) > epsAbs + epsRel * std::max(std::abs(sn_exp), 1.0)) {
        return QStringLiteral("FAIL: characteristic sn line (expected %1, got %2)").arg(sn_exp).arg(ch.sn);
    }

    TurkishLoads::Ts498PartialFactors part;
    const auto combos = TurkishLoads::strCombinationsDepremDisi(ch, part);
    const TurkishLoads::StrCombination *s1 = nullptr;
    const TurkishLoads::StrCombination *s2 = nullptr;
    const TurkishLoads::StrCombination *s3 = nullptr;
    for (const auto &c : combos) {
        if (c.id == QStringLiteral("STR-1")) {
            s1 = &c;
        } else if (c.id == QStringLiteral("STR-2")) {
            s2 = &c;
        } else if (c.id == QStringLiteral("STR-3")) {
            s3 = &c;
        }
    }
    if (!s1 || !s2 || !s3) {
        return QStringLiteral("FAIL: STR-1/2/3 not all present");
    }

    const double gk = ch.dl1 + ch.dl2;
    const double q1_exp = part.gammaG * gk + part.gammaSnow * ch.sn;
    if (std::abs(s1->q_roof_design_kN_m - q1_exp) > 1e-4 + 1e-9 * std::max(std::abs(q1_exp), 1.0)) {
        return QStringLiteral("FAIL: STR-1 q (expected %1, got %2)").arg(q1_exp).arg(s1->q_roof_design_kN_m);
    }
    if (std::abs(s1->w_column_design_kN_m) > 1e-12) {
        return QStringLiteral("FAIL: STR-1 column wind should be 0");
    }

    const double q2_exp = part.gammaG * gk + part.gammaSnow * part.psi0SnowWithWind * ch.sn;
    const double wd2_exp = part.gammaWind * ch.wl;
    if (std::abs(s2->q_roof_design_kN_m - q2_exp) > 1e-4 + 1e-9 * std::max(std::abs(q2_exp), 1.0)) {
        return QStringLiteral("FAIL: STR-2 q roof");
    }
    if (std::abs(s2->w_column_design_kN_m - wd2_exp) > 1e-4) {
        return QStringLiteral("FAIL: STR-2 wd column");
    }

    const double q3_exp = part.gammaG * gk;
    const double wd3_exp = part.gammaWind * ch.wl;
    if (std::abs(s3->q_roof_design_kN_m - q3_exp) > 1e-4 + 1e-9 * std::max(std::abs(q3_exp), 1.0)) {
        return QStringLiteral("FAIL: STR-3 q roof");
    }
    if (std::abs(s3->w_column_design_kN_m - wd3_exp) > 1e-4) {
        return QStringLiteral("FAIL: STR-3 wd column");
    }

    double Nc = 0.0;
    double Mc = 0.0;
    approximateColumnForces(in, s3->q_roof_design_kN_m, s3->w_column_design_kN_m, &Nc, &Mc);
    const double Mc_hand_kNm = (in.columnBaseSupport == ColumnBaseSupport::Pinned)
                                   ? (s3->w_column_design_kN_m * Hc * Hc / 8.0)
                                   : (s3->w_column_design_kN_m * Hc * Hc / 2.0);
    const double Mc_hand_Nm = Mc_hand_kNm * 1000.0;
    if (std::abs(Mc - Mc_hand_Nm) > 1e-3 * std::max(std::abs(Mc_hand_Nm), 1.0)) {
        return QStringLiteral("FAIL: hand column wind moment Mc (approximateColumnForces %1 vs el %2 N·m)")
            .arg(Mc, 0, 'g', 12)
            .arg(Mc_hand_Nm, 0, 'g', 12);
    }

    const auto &hea = SteelCatalog::sectionsForFamily(ColumnFamily::Hea);
    if (hea.empty()) {
        return QStringLiteral("FAIL: empty HEA catalog");
    }
    const RolledISection &sec0 = hea.front();
    const double fyPa = in.fy_MPa * 1e6;
    const double MRd = sec0.Wy_m3 * fyPa / kGammaM0;
    const double uPureBend = columnUtilEC3(0.0, MRd, sec0, fyPa, in.youngModulus_Pa, in);
    if (std::abs(uPureBend - 1.0) > 1e-8) {
        return QStringLiteral("FAIL: columnUtilEC3(0, MRd) should be 1.0 without LTB, got %1")
            .arg(uPureBend, 0, 'g', 17);
    }

    PortalFrameResult geo;
    PortalSolver::buildPortalGeometry(in, geo);
    QString osErr;
    const bool osOk = runOpenSeesStaticAnalysis(in, geo, s3->q_roof_design_kN_m, s3->w_column_design_kN_m, &osErr,
                                                  nullptr);
    const QString pinLectureNote = geo.lectureTrussComparisonNote;
    double M_os_str3 = 0.0;
    if (osOk) {
        for (const auto &m : geo.members) {
            if (m.isTruss) {
                continue;
            }
            M_os_str3 = std::max(M_os_str3, std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm)));
        }
    }

    /** STR-3’te q+wd birlikte; konsol wd·Hc²/2 sınırı yalnızca yalnız rüzgâr yüklemesinde geçerli. */
    double M_wind_only = 0.0;
    QString osWindErr;
    PortalFrameResult geoWind;
    PortalSolver::buildPortalGeometry(in, geoWind);
    const bool osWindOk =
        runOpenSeesStaticAnalysis(in, geoWind, 0.0, s3->w_column_design_kN_m, &osWindErr, nullptr);
    if (osWindOk) {
        for (const auto &m : geoWind.members) {
            if (m.isTruss) {
                continue;
            }
            M_wind_only = std::max(M_wind_only, std::max(std::abs(m.moment_i_Nm), std::abs(m.moment_j_Nm)));
        }
        const double M_ub = Mc_hand_Nm * 1.02 + 100.0;
        if (M_wind_only > M_ub) {
            return QStringLiteral(
                       "FAIL: OpenSees wind-only column |M|max %1 exceeds cantilever upper bound wd·Hc²/2 ~%2 N·m")
                .arg(M_wind_only, 0, 'g', 12)
                .arg(Mc_hand_Nm, 0, 'g', 12);
        }
        if (M_wind_only < 1e-3 && std::abs(s3->w_column_design_kN_m) > 1e-12) {
            return QStringLiteral("FAIL: OpenSees wind-only column moments near zero but wd is non-zero");
        }
    }

    QStringList braceParts;
    for (double f : PortalSolver::columnLateralBraceHeightFractionsEffective(in)) {
        braceParts.append(QString::number(f, 'g', 8));
    }
    const QString braceStr =
        braceParts.isEmpty() ? QStringLiteral("—") : braceParts.join(QLatin1Char(','));

    const double Ncry_sc =
        Ec3Buckling::eulerAxialCriticalForce_N(in.youngModulus_Pa, sec0.Iy_m4, in.columnBucklingKy, Hc);
    const double Ncrz_sc = PortalSolver::columnElasticCriticalForceZz_N(in.youngModulus_Pa, sec0.Iz_m4, in);
    double NcrT_sc = std::numeric_limits<double>::infinity();
    if (sec0.It_m4 > 1e-28 && sec0.Iw_m6 > 1e-35) {
        const double G = in.youngModulus_Pa / (2.0 * (1.0 + 0.3));
        NcrT_sc = Ec3Buckling::eulerTorsionalFlexuralCriticalForce_N(
            in.youngModulus_Pa, G, sec0.Iy_m4, sec0.Iz_m4, sec0.It_m4, sec0.Iw_m6, in.columnBucklingKz, Hc,
            sec0.A_m2);
    }
    const double NcrMin_sc = std::min({Ncry_sc, Ncrz_sc, NcrT_sc});
    auto fmtNkN = [](double n) {
        if (!std::isfinite(n) || n > 0.5e308) {
            return QStringLiteral("∞");
        }
        return QString::number(n / 1000.0, 'g', 8);
    };

    const QString elMcHint = (in.columnBaseSupport == ColumnBaseSupport::Pinned) ? QStringLiteral("wd·Hc²/8")
                                                                                 : QStringLiteral("wd·Hc²/2");
    QString rep = QStringLiteral(
               "OK default self-check\n"
               "  Geometry: W=%1 m, Hc=%2 m, Y=%3 m, fy=%4 MPa\n"
               "  Characteristic line (kN/m): dl1=%5, dl2=%6, gk=%7, sn=%8, wl=%9\n"
               "  STR-3 design: q_roof=%10 kN/m, wd_col=%11 kN/m\n"
               "  Hand column (STR-3): el Mc=%12 kN·m (wind ref: %13); approximateColumnForces Mc=%14 kN·m\n"
               "  columnUtilEC3(0, MRd) on lightest HEA %15: η=%16\n"
               "  OpenSees STR-3 (q+wd): %17; column |M|max=%18 N·m\n"
               "  OpenSees wind-only (q=0, wd): %19; column |M|max=%20 N·m (≤ wd·Hc²/2·1.02)\n"
               "  Burkulma izi: Ky=%21 Kz=%22 tutuluş(h/H)=%23 χ_ord(kolon/makas)=%24/%25 Ncr,y≈%26 kN Ncr,z≈%27 kN Ncr,T≈%28 kN Ncr,min≈%29 kN\n")
        .arg(W, 0, 'g', 6)
        .arg(Hc, 0, 'g', 6)
        .arg(S, 0, 'g', 6)
        .arg(in.fy_MPa, 0, 'g', 6)
        .arg(ch.dl1, 0, 'g', 8)
        .arg(ch.dl2, 0, 'g', 8)
        .arg(gk, 0, 'g', 8)
        .arg(ch.sn, 0, 'g', 8)
        .arg(ch.wl, 0, 'g', 8)
        .arg(s3->q_roof_design_kN_m, 0, 'g', 8)
        .arg(s3->w_column_design_kN_m, 0, 'g', 8)
        .arg(Mc_hand_kNm, 0, 'g', 8)
        .arg(elMcHint)
        .arg(Mc / 1000.0, 0, 'g', 8)
        .arg(sec0.designation)
        .arg(uPureBend, 0, 'g', 12)
        .arg(osOk ? QStringLiteral("ran") : QStringLiteral("skipped (%1)").arg(osErr))
        .arg(M_os_str3, 0, 'g', 12)
        .arg(osWindOk ? QStringLiteral("ran") : QStringLiteral("skipped (%1)").arg(osWindErr))
        .arg(M_wind_only, 0, 'g', 12)
        .arg(in.columnBucklingKy, 0, 'g', 6)
        .arg(in.columnBucklingKz, 0, 'g', 6)
        .arg(braceStr)
        .arg(in.columnBucklingCurveOrdinal)
        .arg(in.trussBucklingCurveOrdinal)
        .arg(fmtNkN(Ncry_sc))
        .arg(fmtNkN(Ncrz_sc))
        .arg(fmtNkN(NcrT_sc))
        .arg(fmtNkN(NcrMin_sc));
    if (!pinLectureNote.isEmpty()) {
        rep += QLatin1Char('\n');
        rep += pinLectureNote;
    }
    return rep;
}
