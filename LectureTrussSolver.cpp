#include "LectureTrussSolver.h"

#include "PortalSolver.h"
#include "SectionOptimizer.h"
#include "SteelCatalog.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace {

constexpr double kN_to_N = 1000.0;

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

void trussMemberAxialArea_m2(const PortalFrameInput &in, const SectionOptimizationResult *sizing,
                             const MemberResult &member, double *Aout)
{
    const double A_nom = in.steelArea_m2;

    if (!sizing) {
        *Aout = A_nom;
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
        return;
    }

    const auto sec = SteelCatalog::tryGetDoubleAngle2L(prof);
    if (!sec.has_value()) {
        *Aout = A_nom;
        return;
    }

    *Aout = sec->A_total_m2;
}

void roofLineNodalFy_N(const PortalFrameInput &in, const PortalFrameResult &geo, double q_roof_design_kN_per_m,
                       std::map<int, std::pair<double, double>> *L)
{
    const double w_N_m = q_roof_design_kN_per_m * kN_to_N;
    for (const auto &m : geo.members) {
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
        (*L)[m.nodeI].second -= half;
        (*L)[m.nodeJ].second -= half;
    }
}

bool gaussJordanSolve(size_t n, std::vector<double> &A, std::vector<double> &b)
{
    for (size_t k = 0; k < n; ++k) {
        size_t piv = k;
        double best = std::fabs(A[k * n + k]);
        for (size_t r = k + 1; r < n; ++r) {
            const double v = std::fabs(A[r * n + k]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1e-20) {
            return false;
        }
        if (piv != k) {
            for (size_t j = 0; j < n; ++j) {
                std::swap(A[k * n + j], A[piv * n + j]);
            }
            std::swap(b[k], b[piv]);
        }
        const double div = A[k * n + k];
        for (size_t j = k; j < n; ++j) {
            A[k * n + j] /= div;
        }
        b[k] /= div;
        for (size_t r = 0; r < n; ++r) {
            if (r == k) {
                continue;
            }
            const double f = A[r * n + k];
            if (f == 0.0) {
                continue;
            }
            for (size_t j = 0; j < n; ++j) {
                A[r * n + j] -= f * A[k * n + j];
            }
            b[r] -= f * b[k];
        }
    }
    return true;
}

} // namespace

bool LectureTrussSolver::compareAxialVsOpenSees(const PortalFrameInput &in, const PortalFrameResult &geo,
                                                double q_roof_design_kN_per_m,
                                                double w_column_design_kN_per_m,
                                                const SectionOptimizationResult *sizing, QString *outSummary,
                                                LectureTrussComparisonStats *outStats)
{
    if (outSummary) {
        outSummary->clear();
    }
    if (outStats) {
        *outStats = {};
    }

    const size_t nn = geo.nodes.size();
    if (nn < 2 || geo.members.empty()) {
        if (outSummary) {
            *outSummary = QStringLiteral("Pin-eksen karşılaştırma: geometri yetersiz.");
        }
        return false;
    }

    int maxTag = 0;
    for (const auto &n : geo.nodes) {
        maxTag = std::max(maxTag, n.tag);
    }
    if (maxTag <= 0) {
        if (outSummary) {
            *outSummary = QStringLiteral("Pin-eksen karşılaştırma: düğüm etiketi yok.");
        }
        return false;
    }
    std::vector<int> tagToIdx(static_cast<size_t>(maxTag) + 1u, -1);
    for (size_t i = 0; i < nn; ++i) {
        const int t = geo.nodes[i].tag;
        if (t >= 0 && static_cast<size_t>(t) < tagToIdx.size()) {
            tagToIdx[static_cast<size_t>(t)] = static_cast<int>(i);
        }
    }

    const size_t ndof = 2 * nn;
    std::vector<char> fixed(ndof, 0);
    const double W = in.spanWidth_m;
    for (const auto &n : geo.nodes) {
        if (isBaseNode(geo, n.tag, W) || isGussetFootNode(geo, n.tag)) {
            if (n.tag < 0 || static_cast<size_t>(n.tag) >= tagToIdx.size()) {
                continue;
            }
            const int idx = tagToIdx[static_cast<size_t>(n.tag)];
            if (idx < 0) {
                continue;
            }
            fixed[static_cast<size_t>(2 * idx)] = 1;
            fixed[static_cast<size_t>(2 * idx + 1)] = 1;
        }
    }

    std::vector<int> mapFullToFree(ndof, -1);
    size_t nf = 0;
    for (size_t d = 0; d < ndof; ++d) {
        if (!fixed[d]) {
            mapFullToFree[d] = static_cast<int>(nf++);
        }
    }
    if (nf == 0) {
        if (outSummary) {
            *outSummary = QStringLiteral("Pin-eksen karşılaştırma: serbest DOF kalmadı.");
        }
        return false;
    }

    std::vector<double> Kred(nf * nf, 0.0);
    std::vector<double> Rred(nf, 0.0);

    const double E = in.youngModulus_Pa;
    const double Ac = in.steelArea_m2;

    for (const auto &m : geo.members) {
        const int ti = m.nodeI;
        const int tj = m.nodeJ;
        if (ti < 0 || tj < 0 || static_cast<size_t>(ti) >= tagToIdx.size()
            || static_cast<size_t>(tj) >= tagToIdx.size()) {
            continue;
        }
        const int ni = tagToIdx[static_cast<size_t>(ti)];
        const int nj = tagToIdx[static_cast<size_t>(tj)];
        if (ni < 0 || nj < 0) {
            continue;
        }
        double xi = 0.0;
        double yi = 0.0;
        double xj = 0.0;
        double yj = 0.0;
        if (!nodeCoords(geo, ti, xi, yi) || !nodeCoords(geo, tj, xj, yj)) {
            continue;
        }
        const double dx = xj - xi;
        const double dy = yj - yi;
        const double Lm = std::hypot(dx, dy);
        if (Lm < 1e-12) {
            continue;
        }
        double Asec = Ac;
        if (m.isTruss) {
            trussMemberAxialArea_m2(in, sizing, m, &Asec);
        }
        const double kl = E * Asec / Lm;
        const double c = dx / Lm;
        const double s = dy / Lm;
        const size_t d0 = static_cast<size_t>(2 * ni);
        const size_t d1 = d0 + 1;
        const size_t d2 = static_cast<size_t>(2 * nj);
        const size_t d3 = d2 + 1;
        const size_t dof[4] = {d0, d1, d2, d3};
        const double k11 = c * c * kl;
        const double k12 = c * s * kl;
        const double k22 = s * s * kl;
        const double kmat[4][4] = {
            {k11, k12, -k11, -k12},
            {k12, k22, -k12, -k22},
            {-k11, -k12, k11, k12},
            {-k12, -k22, k12, k22},
        };
        for (int a = 0; a < 4; ++a) {
            for (int b = 0; b < 4; ++b) {
                const size_t ga = dof[a];
                const size_t gb = dof[b];
                if (fixed[ga] || fixed[gb]) {
                    continue;
                }
                const int ma = mapFullToFree[ga];
                const int mb = mapFullToFree[gb];
                Kred[static_cast<size_t>(ma) * nf + static_cast<size_t>(mb)] += kmat[a][b];
            }
        }
    }

    std::map<int, std::pair<double, double>> Lmap;
    roofLineNodalFy_N(in, geo, q_roof_design_kN_per_m, &Lmap);
    for (const auto &kv : Lmap) {
        const int t = kv.first;
        if (t < 0 || static_cast<size_t>(t) >= tagToIdx.size()) {
            continue;
        }
        const int idx = tagToIdx[static_cast<size_t>(t)];
        if (idx < 0) {
            continue;
        }
        const size_t gfy = static_cast<size_t>(2 * idx + 1);
        if (!fixed[gfy]) {
            Rred[static_cast<size_t>(mapFullToFree[gfy])] += kv.second.second;
        }
    }

    {
        std::vector<double> diag(nf, 0.0);
        for (size_t i = 0; i < nf; ++i) {
            diag[i] = Kred[i * nf + i];
        }
        std::sort(diag.begin(), diag.end());
        const double medDiag = diag[nf / 2];
        const double stab = std::max(medDiag * 1e-10, 1e-3);
        for (size_t i = 0; i < nf; ++i) {
            Kred[i * nf + i] += stab;
        }
    }

    std::vector<double> Awork = Kred;
    std::vector<double> ured = Rred;
    if (!gaussJordanSolve(nf, Awork, ured)) {
        if (outSummary) {
            *outSummary =
                QStringLiteral("Pin-eksen makas modeli tekil / çözülemedi (OpenSees beam modeli ile doğrudan örtüşmez).");
        }
        return false;
    }

    std::vector<double> u(ndof, 0.0);
    for (size_t d = 0; d < ndof; ++d) {
        if (!fixed[d]) {
            u[d] = ured[static_cast<size_t>(mapFullToFree[d])];
        }
    }

    LectureTrussComparisonStats st;
    for (const auto &m : geo.members) {
        if (!m.isTruss) {
            continue;
        }
        const int ti = m.nodeI;
        const int tj = m.nodeJ;
        if (ti < 0 || tj < 0 || static_cast<size_t>(ti) >= tagToIdx.size()
            || static_cast<size_t>(tj) >= tagToIdx.size()) {
            continue;
        }
        const int ni = tagToIdx[static_cast<size_t>(ti)];
        const int nj = tagToIdx[static_cast<size_t>(tj)];
        if (ni < 0 || nj < 0) {
            continue;
        }
        double xi = 0.0;
        double yi = 0.0;
        double xj = 0.0;
        double yj = 0.0;
        if (!nodeCoords(geo, ti, xi, yi) || !nodeCoords(geo, tj, xj, yj)) {
            continue;
        }
        const double dx = xj - xi;
        const double dy = yj - yi;
        const double Lm = std::hypot(dx, dy);
        if (Lm < 1e-9) {
            continue;
        }
        double Asec = Ac;
        trussMemberAxialArea_m2(in, sizing, m, &Asec);
        const double kl = E * Asec / Lm;
        const double c = dx / Lm;
        const double s = dy / Lm;
        const double uxi = u[static_cast<size_t>(2 * ni)];
        const double uyi = u[static_cast<size_t>(2 * ni + 1)];
        const double uxj = u[static_cast<size_t>(2 * nj)];
        const double uyj = u[static_cast<size_t>(2 * nj + 1)];
        const double elong = (uxj - uxi) * c + (uyj - uyi) * s;
        const double Nlect = kl * elong;
        const double Nos = m.axial_N;
        const double d1 = std::abs(Nlect - Nos);
        const double d2 = std::abs(-Nlect - Nos);
        const double d = std::min(d1, d2);
        st.maxAbsDiffN = std::max(st.maxAbsDiffN, d);
        if (std::abs(Nos) > 1.0) {
            st.maxRelDiff = std::max(st.maxRelDiff, d / std::abs(Nos));
        }
        ++st.nCompared;
    }

    if (outStats) {
        *outStats = st;
    }

    if (outSummary) {
        *outSummary =
            QStringLiteral("Pin-eksen (ders notu tipi) makas: %1 çubukta |ΔN|_max = %2 kN")
                .arg(st.nCompared)
                .arg(st.maxAbsDiffN / kN_to_N, 0, 'g', 5);
        if (std::abs(w_column_design_kN_per_m) > 1e-12) {
            *outSummary += QStringLiteral(
                " — bu modelde kolon rüzgârı yok; OpenSees’te beam-column + eleLoad ile fark büyüyebilir.");
        }
        *outSummary += QStringLiteral(
            " [Pim çerçevede yatay tekillik: K’ya çok küçük köşegen stabilizasyon eklendi.]");
    }
    return true;
}

static bool trussMakasGroupsMatch(const SectionOptimizationResult &a, const SectionOptimizationResult &b)
{
    return a.trussTopChord2xL == b.trussTopChord2xL && a.trussBottomChord2xL == b.trussBottomChord2xL
        && a.trussEdgePost2xL == b.trussEdgePost2xL && a.trussWebB2xL == b.trussWebB2xL
        && a.trussWebC2xL == b.trussWebC2xL && a.trussWebD2xL == b.trussWebD2xL;
}

QString LectureTrussSolver::formatTrussSectionsCompareBlock(const SectionOptimizationResult *openSeesAndPinStepSizing,
                                                            const SectionOptimizationResult *envelopeAfterPhysical)
{
    auto block = [](const SectionOptimizationResult *p) -> QString {
        if (!p) {
            return QStringLiteral("  (veri yok)\n");
        }
        return QStringLiteral(
                   "  Üst hat: %1\n  Alt hat (+45° gusset): %2\n  Kenar post: %3\n"
                   "  Köşegen B: %4\n  Köşegen C: %5\n  Köşegen D: %6\n")
            .arg(p->trussTopChord2xL, p->trussBottomChord2xL, p->trussEdgePost2xL, p->trussWebB2xL, p->trussWebC2xL,
                 p->trussWebD2xL);
    };

    QString r = QStringLiteral(
        "\n\n── Makas 2×L kesitleri (karşılaştırma: ders notu pin A = OpenSees Tcl’deki alan) ──\n");
    r += QStringLiteral("1) Birinci zarf (OpenSees + pin-eksen ortak):\n");
    r += block(openSeesAndPinStepSizing);
    r += QStringLiteral("\n2) STR zarfı (fiziksel koşular sonrası EC3 seçimi):\n");
    r += block(envelopeAfterPhysical);
    if (openSeesAndPinStepSizing && envelopeAfterPhysical
        && trussMakasGroupsMatch(*openSeesAndPinStepSizing, *envelopeAfterPhysical)) {
        r += QStringLiteral("\n→ Makas grupları iki adımda da aynı.\n");
    } else if (openSeesAndPinStepSizing && envelopeAfterPhysical) {
        r += QStringLiteral(
            "\n→ Fark olabilir: pin kuvvet karşılaştırması 1. zarf alanlarını kullanır; zarf satırı tüm STR "
            "üzerinden yeniden seçilir.\n");
    }
    return r;
}
