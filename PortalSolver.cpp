#include "PortalSolver.h"
#include "Buckling.h"
#include "OpenSeesRunner.h"
#include "LectureTrussSolver.h"
#include "SteelCatalog.h"
#include "TurkishLoads.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/** Üst hat panel etiketi (çizim): ilk k üst hat segmenti 1…k. */
constexpr int kWarrenTopChordLabeledPanels = 4;

/** Global köşegen sırası w’ye göre sanal bölge (tam açıklık çerçevesinde tonaj optimizasyonu). */
uint8_t webOptZoneFromGlobalSeq(int w, int wtot)
{
    if (wtot <= 0 || w < 0 || w >= wtot) {
        return 0;
    }
    if (w < 2) {
        return 0;
    }
    if (wtot > 2 && w == wtot - 2) {
        return 3;
    }
    if (w < 8) {
        return 1;
    }
    if (w < 16) {
        return 2;
    }
    return 2;
}

QString formatManualStaticCheckBlock(const PortalFrameInput &in, const std::vector<TurkishLoads::StrCombination> &combos,
                                     const std::vector<PortalFrameResult> &results)
{
    if (combos.empty() || combos.size() != results.size()) {
        return QString();
    }

    const double W = in.spanWidth_m;
    const double H = in.columnHeight_m;
    const double halfSpan = 0.5 * W;
    const double Ha = PortalSolver::trussApexHeight_m(in);
    const double rise = std::max(Ha - H, 1e-6);
    const double trussDepthApprox =
        std::max(0.5 * (W / 40.0 + W / 11.0), std::max(0.22 * rise, 0.05));
    const bool pinnedBase = in.columnBaseSupport == ColumnBaseSupport::Pinned;

    QString s = QStringLiteral(
        "\n\nElle kontrol (yaklaşık denge, kesit seçimini yorumlamak için):\n"
        "  Çatı q değeri yatay izdüşüm başına düşey hat yükü kabul edilir. Rkol≈q·W/2; "
        "Mçatı≈q·(W/2)^2/8; Mkol,rüz≈wd·H^2/%1.\n")
                    .arg(pinnedBase ? QStringLiteral("8") : QStringLiteral("2"));

    for (size_t ci = 0; ci < combos.size(); ++ci) {
        const auto &co = combos[ci];
        const auto &r = results[ci];

        double osColN_kN = 0.0;
        double osColM_kNm = 0.0;
        double osTrussN_kN = 0.0;
        double osRafterM_kNm = 0.0;
        for (const auto &m : r.members) {
            const double Mi = std::abs(m.moment_i_Nm) / 1000.0;
            const double Mj = std::abs(m.moment_j_Nm) / 1000.0;
            if (m.trussRole == TrussMemberRole::Column) {
                osColN_kN = std::max(osColN_kN, std::abs(m.axial_N) / 1000.0);
                osColM_kNm = std::max(osColM_kNm, std::max(Mi, Mj));
            } else if (m.trussRole == TrussMemberRole::RafterBeam) {
                osRafterM_kNm = std::max(osRafterM_kNm, std::max(Mi, Mj));
            } else if (m.isTruss) {
                osTrussN_kN = std::max(osTrussN_kN, std::abs(m.axial_N) / 1000.0);
            }
        }

        const double q = co.q_roof_design_kN_m;
        const double wd = co.w_column_design_kN_m;
        const double roofTotal_kN = q * W;
        const double reactionPerColumn_kN = q * halfSpan;
        const double roofMomentHalf_kNm = q * halfSpan * halfSpan / 8.0;
        const double windMoment_kNm = pinnedBase ? (wd * H * H / 8.0) : (wd * H * H / 2.0);
        const double trussAxialApprox_kN = roofMomentHalf_kNm / std::max(trussDepthApprox, 0.05) * 1.05;

        s += QStringLiteral(
                 "  %1: q=%2 kN/m, wd=%3 kN/m | el: Vçatı=%4 kN, Rkol=%5 kN, "
                 "Mçatı=%6 kN·m, Mkol,w=%7 kN·m, Nmakas≈%8 kN | OpenSees zarf: "
                 "Nkol=%9 kN, Mkol=%10 kN·m")
                 .arg(co.id)
                 .arg(q, 0, 'g', 6)
                 .arg(wd, 0, 'g', 6)
                 .arg(roofTotal_kN, 0, 'g', 6)
                 .arg(reactionPerColumn_kN, 0, 'g', 6)
                 .arg(roofMomentHalf_kNm, 0, 'g', 6)
                 .arg(windMoment_kNm, 0, 'g', 6)
                 .arg(trussAxialApprox_kN, 0, 'g', 6)
                 .arg(osColN_kN, 0, 'g', 6)
                 .arg(osColM_kNm, 0, 'g', 6);
        if (in.trussMemberSectionForm == 3) {
            s += QStringLiteral(", Mmahya=%1 kN·m").arg(osRafterM_kNm, 0, 'g', 6);
        } else {
            s += QStringLiteral(", Nmakas=%1 kN").arg(osTrussN_kN, 0, 'g', 6);
        }
        s += QLatin1Char('\n');
    }

    s += QStringLiteral(
        "  Not: Elle değerler hızlı denge kontrolüdür; OpenSees çerçeve sürekliliği, kolon rijitliği ve eleman "
        "eksenlerini içerdiği için bire bir aynı çıkması beklenmez.\n");
    return s;
}

} // namespace

double PortalSolver::trussApexHeight_m(const PortalFrameInput &input)
{
    const double L = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    const double hEdge = L / 40.0;
    return (Hc - hEdge) + L / 11.0;
}

double PortalSolver::rafterRoofUniformLocalY_N_per_m(double q_roof_design_N_per_m, double xi, double yi, double xj,
                                                     double yj)
{
    (void)yi;
    (void)yj;
    const double dx = xj - xi;
    const double dy = yj - yi;
    const double L = std::hypot(dx, dy);
    if (L < 1e-12 || std::abs(q_roof_design_N_per_m) < 1e-18) {
        return 0.0;
    }
    const double w_vert_per_m_arc = q_roof_design_N_per_m * std::abs(dx) / L;
    return -w_vert_per_m_arc * (dx / L);
}

bool PortalSolver::validate(const PortalFrameInput &in, QString *err) const
{
    if (in.spanWidth_m <= 0.0 || in.columnHeight_m <= 0.0) {
        if (err) {
            *err = QStringLiteral("Span and column height must be positive.");
        }
        return false;
    }
    if (in.trussMemberSectionForm != 3 && in.columnHeight_m <= in.spanWidth_m / 40.0 + 1e-12) {
        if (err) {
            *err = QStringLiteral("Kolon yüksekliği, tam açıklık L üzerinden L/40 alt hat için yeterli olmalıdır.");
        }
        return false;
    }
    if (in.trussPanelsPerSide < 1) {
        if (err) {
            *err = QStringLiteral("Number of truss panels per side must be at least 1.");
        }
        return false;
    }
    if (in.trussAxisSpacingY_m <= 0.0) {
        if (err) {
            *err = QStringLiteral("Makas aks aralığı Y pozitif olmalıdır.");
        }
        return false;
    }
    if (in.steelArea_m2 <= 0.0 || in.youngModulus_Pa <= 0.0) {
        if (err) {
            *err = QStringLiteral("Steel A and E must be positive.");
        }
        return false;
    }
    if (in.steelInertia_m4 <= 0.0) {
        if (err) {
            *err = QStringLiteral("Steel I must be positive for beam-columns.");
        }
        return false;
    }
    if (in.fy_MPa <= 0.0) {
        if (err) {
            *err = QStringLiteral("fy must be positive.");
        }
        return false;
    }
    if (in.columnFamilyIndex < 0 || in.columnFamilyIndex > 2) {
        if (err) {
            *err = QStringLiteral("Column family index must be 0 (HEA), 1 (HEB), or 2 (IPE).");
        }
        return false;
    }
    if (in.trussMemberSectionForm < 0 || in.trussMemberSectionForm > 3) {
        if (err) {
            *err = QStringLiteral("Makas kesit formu 0 (2×L), 1 (HEA), 2 (HEB) veya 3 (IPE) olmalıdır.");
        }
        return false;
    }
    if (static_cast<int>(in.columnBaseSupport) < 0 || static_cast<int>(in.columnBaseSupport) > 1) {
        if (err) {
            *err = QStringLiteral("Column base support must be fixed (0) or pinned (1).");
        }
        return false;
    }
    if (in.columnBucklingKy <= 0.05 || in.columnBucklingKy > 20.0) {
        if (err) {
            *err = QStringLiteral("Column buckling Ky must be in (0.05, 20].");
        }
        return false;
    }
    if (in.columnBucklingKz <= 0.05 || in.columnBucklingKz > 20.0) {
        if (err) {
            *err = QStringLiteral("Column buckling Kz must be in (0.05, 20].");
        }
        return false;
    }
    if (in.columnBucklingCurveOrdinal < 0 || in.columnBucklingCurveOrdinal > 4 || in.trussBucklingCurveOrdinal < 0
        || in.trussBucklingCurveOrdinal > 4) {
        if (err) {
            *err = QStringLiteral("Buckling imperfection curve ordinal must be 0…4 (A0…D).");
        }
        return false;
    }
    for (double f : in.columnLateralBraceHeightFractions) {
        if (f <= 1e-6 || f >= 1.0 - 1e-6) {
            if (err) {
                *err = QStringLiteral("Column lateral brace height fractions must be strictly between 0 and 1.");
            }
            return false;
        }
    }
    for (double f : in.columnZzBucklingBraceHeightFractions) {
        if (f <= 1e-6 || f >= 1.0 - 1e-6) {
            if (err) {
                *err = QStringLiteral("Column z-z buckling brace height fractions must be strictly between 0 and 1.");
            }
            return false;
        }
    }
    for (double f : in.columnLtbBottomFlangeBraceHeightFractions) {
        if (f <= 1e-6 || f >= 1.0 - 1e-6) {
            if (err) {
                *err = QStringLiteral("Column LTB bottom-flange brace height fractions must be strictly between 0 and 1.");
            }
            return false;
        }
    }
    for (double f : in.columnLtbTopFlangeBraceHeightFractions) {
        if (f <= 1e-6 || f >= 1.0 - 1e-6) {
            if (err) {
                *err = QStringLiteral("Column LTB top-flange brace height fractions must be strictly between 0 and 1.");
            }
            return false;
        }
    }
    for (double f : in.trussRestraintFractions) {
        if (f <= 1e-6 || f >= 1.0 - 1e-6) {
            if (err) {
                *err = QStringLiteral("Truss restraint fractions must be strictly between 0 and 1.");
            }
            return false;
        }
    }
    if (in.columnLtbCurveOrdinal < 0 || in.columnLtbCurveOrdinal > 4) {
        if (err) {
            *err = QStringLiteral("Column LTB curve ordinal must be 0…4 (A0…D).");
        }
        return false;
    }
    return true;
}

PortalFrameResult PortalSolver::analyze(const PortalFrameInput &input) const
{
    PortalFrameResult out;
    QString err;
    if (!validate(input, &err)) {
        out.ok = false;
        out.errorMessage = err;
        return out;
    }

    const auto ch = input.lineLoadsCharacteristic();
    const TurkishLoads::Ts498PartialFactors part;
    const auto combos = TurkishLoads::strCombinationsDepremDisi(ch, part);

    PortalFrameResult base;
    buildPortalGeometry(input, base);

    QString osErr;
    std::vector<PortalFrameResult> comboResults;
    comboResults.reserve(combos.size());
    for (const auto &co : combos) {
        PortalFrameResult r = base;
        if (!runOpenSeesStaticAnalysis(input, r, co.q_roof_design_kN_m, co.w_column_design_kN_m, &osErr,
                                        nullptr)) {
            out.ok = false;
            out.errorMessage = osErr;
            return out;
        }
        comboResults.push_back(std::move(r));
    }

    const auto fam = static_cast<ColumnFamily>(input.columnFamilyIndex);
    const SectionOptimizationResult pass1Design = optimizeSectionsEnvelope(input, comboResults, combos, input.fy_MPa, fam);

    std::vector<PortalFrameResult> comboResultsPhysical;
    comboResultsPhysical.reserve(combos.size());
    for (const auto &co : combos) {
        PortalFrameResult r = base;
        if (!runOpenSeesStaticAnalysis(input, r, co.q_roof_design_kN_m, co.w_column_design_kN_m, &osErr,
                                        &pass1Design)) {
            out.ok = false;
            out.errorMessage = osErr;
            return out;
        }
        comboResultsPhysical.push_back(std::move(r));
    }

    out = comboResultsPhysical.front();
    out.sectionDesign = optimizeSectionsEnvelope(input, comboResultsPhysical, combos, input.fy_MPa, fam);

    out.ok = true;
    QString combIds;
    for (const auto &c : combos) {
        if (!combIds.isEmpty()) {
            combIds += QStringLiteral(", ");
        }
        combIds += c.id;
    }
    out.errorMessage =
        TurkishLoads::formatRegulatorySummary(ch, part)
        + QStringLiteral(" [Analizde kullanılan makas aks Y=%1 m — hat yükleri buna göre]")
              .arg(input.trussAxisSpacingY_m, 0, 'g', 10)
        + QStringLiteral(" OpenSees STR zarf: %1. %2 Kolon: %3, makas: %4.")
              .arg(combIds)
              .arg(out.sectionDesign.envelopeNote)
              .arg(out.sectionDesign.columnProfile)
              .arg(out.sectionDesign.trussProfile2xL)
        + formatManualStaticCheckBlock(input, combos, comboResultsPhysical);
    if (input.trussMemberSectionForm != 3) {
        out.lectureTrussComparisonNote += LectureTrussSolver::formatTrussSectionsCompareBlock(&pass1Design,
                                                                                            &out.sectionDesign);
    }
    return out;
}

void PortalSolver::buildPortalGeometry(const PortalFrameInput &input, PortalFrameResult &out)
{
    const double W = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    const double hEdge = W / 40.0;
    const double Ha = trussApexHeight_m(input);
    const int N = input.trussPanelsPerSide;

    out.nodes.clear();
    out.members.clear();

    int nextTag = 1;
    auto addNode = [&](double x, double y) {
        NodeResult nr;
        nr.tag = nextTag++;
        nr.x = x;
        nr.y = y;
        out.nodes.push_back(nr);
        return nr.tag;
    };

    auto addMember = [&](int i, int j, bool truss, bool roofLine, TrussMemberRole role, uint8_t tcLbl = 0,
                         bool tcLeft = true, uint8_t webOptZone = 0) {
        MemberResult m;
        m.tag = static_cast<int>(out.members.size()) + 1;
        m.nodeI = i;
        m.nodeJ = j;
        m.isTruss = truss;
        m.carriesRoofLineLoad = roofLine;
        m.trussRole = role;
        m.topChordLabelNumber = tcLbl;
        m.topChordLabelOnLeft = tcLeft;
        m.webOptZone = (truss && role == TrussMemberRole::Web) ? webOptZone : 0;
        out.members.push_back(m);
    };

    const int idLeftBase = addNode(0.0, 0.0);
    const int idRightBase = addNode(W, 0.0);
    /** Üst hat kolon tepesi (Hc); alt hat tam açıklık L=W için L/40 aşağıda; köşede düşey çubuk yok (ilk çapraz alttan). */
    const int idLeftEave = addNode(0.0, Hc);
    const int idRightEave = addNode(W, Hc);

    addMember(idLeftBase, idLeftEave, false, false, TrussMemberRole::Column);
    addMember(idRightBase, idRightEave, false, false, TrussMemberRole::Column);

    /** IPE pütrel: Warren yok; yalnızca mahya = iki yarım elastik kiriş (düz çatı hattı). */
    if (input.trussMemberSectionForm == 3) {
        const int idApexIpe = addNode(0.5 * W, Ha);
        addMember(idLeftEave, idApexIpe, false, true, TrussMemberRole::RafterBeam);
        addMember(idApexIpe, idRightEave, false, true, TrussMemberRole::RafterBeam);
        return;
    }

    const int idLeftBottom = addNode(0.0, Hc - hEdge);
    const int idRightBottom = addNode(W, Hc - hEdge);
    const int idCrown = addNode(0.5 * W, Hc - hEdge);
    const int idApex = addNode(0.5 * W, Ha);

    std::vector<int> bottomLeft;
    std::vector<int> bottomRight;
    std::vector<int> topLeft;
    std::vector<int> topRight;

    bottomLeft.reserve(static_cast<size_t>(N + 1));
    bottomRight.reserve(static_cast<size_t>(N + 1));
    topLeft.reserve(static_cast<size_t>(N + 1));
    topRight.reserve(static_cast<size_t>(N + 1));

    bottomLeft.push_back(idLeftBottom);
    for (int j = 1; j < N; ++j) {
        const double s = static_cast<double>(j) / static_cast<double>(N);
        const double x = s * 0.5 * W;
        bottomLeft.push_back(addNode(x, Hc - hEdge));
    }
    bottomLeft.push_back(idCrown);

    bottomRight.push_back(idRightBottom);
    for (int j = 1; j < N; ++j) {
        const double s = static_cast<double>(j) / static_cast<double>(N);
        const double x = W - s * 0.5 * W;
        bottomRight.push_back(addNode(x, Hc - hEdge));
    }
    bottomRight.push_back(idCrown);

    topLeft.push_back(idLeftEave);
    for (int j = 1; j < N; ++j) {
        const double s = static_cast<double>(j) / static_cast<double>(N);
        const double x = s * 0.5 * W;
        const double y = Hc + s * (Ha - Hc);
        topLeft.push_back(addNode(x, y));
    }
    topLeft.push_back(idApex);

    topRight.push_back(idRightEave);
    for (int j = 1; j < N; ++j) {
        const double s = static_cast<double>(j) / static_cast<double>(N);
        const double x = W - s * 0.5 * W;
        const double y = Hc + s * (Ha - Hc);
        topRight.push_back(addNode(x, y));
    }
    topRight.push_back(idApex);

    /**
     * Köşe gusset: kolon üzerinde alt hattan Δy aşağıda başlar, alt başlık hattında Δx içeri 45° çapraz ile köşeye bağlanır
     * (Δx = Δy = min(1 m, ilk alt hat panel yarı genişliği − tolerans)).
     */
    const double yBotCorner = Hc - hEdge;
    const double firstBottomHalfWidth = (N >= 1) ? (0.5 * W / static_cast<double>(N)) : (0.5 * W);
    constexpr double kGussetLegPreferred_m = 1.0;
    const double leg =
        std::clamp(kGussetLegPreferred_m, 0.05, std::max(0.05, firstBottomHalfWidth - 0.02));
    const double yGusCol = yBotCorner - leg;
    const int idLeftGusCol = addNode(0.0, std::max(yGusCol, 1e-3));
    const int idLeftGusTip = addNode(leg, yBotCorner);
    const int idRightGusCol = addNode(W, std::max(yGusCol, 1e-3));
    const int idRightGusTip = addNode(W - leg, yBotCorner);

    for (int j = 0; j < N; ++j) {
        addMember(bottomLeft[static_cast<size_t>(j)], bottomLeft[static_cast<size_t>(j + 1)], true, false,
                  TrussMemberRole::ChordBottom);
        const uint8_t lbl =
            (j < kWarrenTopChordLabeledPanels) ? static_cast<uint8_t>(j + 1) : static_cast<uint8_t>(0);
        addMember(topLeft[static_cast<size_t>(j)], topLeft[static_cast<size_t>(j + 1)], true, true,
                  TrussMemberRole::ChordTop, lbl, true);
    }
    for (int j = 0; j < N; ++j) {
        addMember(bottomRight[static_cast<size_t>(j)], bottomRight[static_cast<size_t>(j + 1)], true, false,
                  TrussMemberRole::ChordBottom);
        const uint8_t lbl =
            (j < kWarrenTopChordLabeledPanels) ? static_cast<uint8_t>(j + 1) : static_cast<uint8_t>(0);
        addMember(topRight[static_cast<size_t>(j)], topRight[static_cast<size_t>(j + 1)], true, true,
                  TrussMemberRole::ChordTop, lbl, false);
    }

    if (leg > 1e-6 && yGusCol > 1e-6 && yGusCol + 1e-9 < yBotCorner) {
        addMember(idLeftGusCol, idLeftGusTip, true, false, TrussMemberRole::GussetStrip);
        addMember(idRightGusCol, idRightGusTip, true, false, TrussMemberRole::GussetStrip);
    }

    /** Tam açıklıkta 2N köşegen var; bölge indeksi simetrik çiftler için aynı olmalı (sol j ↔ sağ j). */
    const int wtot = 2 * N;
    for (int j = 0; j < N; ++j) {
        const bool zig = (j % 2) == 0;
        const uint8_t zWeb = webOptZoneFromGlobalSeq(2 * j, wtot);
        if (zig) {
            addMember(bottomLeft[static_cast<size_t>(j)], topLeft[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zWeb);
            addMember(bottomRight[static_cast<size_t>(j)], topRight[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zWeb);
        } else {
            addMember(topLeft[static_cast<size_t>(j)], bottomLeft[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zWeb);
            addMember(topRight[static_cast<size_t>(j)], bottomRight[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zWeb);
        }
    }
}

std::vector<double> PortalSolver::columnTrussDerivedBraceHeightFractions(const PortalFrameInput &input)
{
    std::vector<double> out;
    if (input.trussMemberSectionForm == 3) {
        return out;
    }
    const double W = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    if (Hc < 1e-9 || W < 1e-9) {
        return out;
    }
    const double hEdge = W / 40.0;
    const double yBotCorner = Hc - hEdge;
    const double fBottom = yBotCorner / Hc;
    if (fBottom > 1e-6 && fBottom < 1.0 - 1e-6) {
        out.push_back(fBottom);
    }
    const int N = input.trussPanelsPerSide;
    const double firstBottomHalfWidth = (N >= 1) ? (0.5 * W / static_cast<double>(N)) : (0.5 * W);
    constexpr double kGussetLegPreferred_m = 1.0;
    const double leg =
        std::clamp(kGussetLegPreferred_m, 0.05, std::max(0.05, firstBottomHalfWidth - 0.02));
    const double yGusCol = yBotCorner - leg;
    if (leg > 1e-6 && yGusCol > 1e-6 && yGusCol + 1e-9 < yBotCorner) {
        const double fGus = yGusCol / Hc;
        if (fGus > 1e-6 && fGus < 1.0 - 1e-6 && std::abs(fGus - fBottom) > 1e-9) {
            out.push_back(fGus);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<double> PortalSolver::columnLateralBraceHeightFractionsEffective(const PortalFrameInput &input)
{
    std::vector<double> fr = input.columnLateralBraceHeightFractions;
    if (input.columnLateralBraceFromTrussGeometry) {
        const std::vector<double> geo = columnTrussDerivedBraceHeightFractions(input);
        for (const double g : geo) {
            fr.push_back(g);
        }
    }
    fr.erase(std::remove_if(fr.begin(), fr.end(),
                            [](double x) { return x <= 1e-6 || x >= 1.0 - 1e-6; }),
             fr.end());
    std::sort(fr.begin(), fr.end());
    fr.erase(std::unique(fr.begin(), fr.end()), fr.end());
    return fr;
}

double PortalSolver::columnElasticCriticalForceZz_N(double E_Pa, double Iz_m4, const PortalFrameInput &input)
{
    const double H = input.columnHeight_m;
    if (H < 1e-12 || Iz_m4 < 1e-24) {
        return std::numeric_limits<double>::max();
    }
    std::vector<double> fr;
    if (!input.columnZzBucklingBraceHeightFractions.empty()) {
        fr = input.columnZzBucklingBraceHeightFractions;
        fr.erase(std::remove_if(fr.begin(), fr.end(),
                                [](double x) { return x <= 1e-6 || x >= 1.0 - 1e-6; }),
                 fr.end());
        std::sort(fr.begin(), fr.end());
        fr.erase(std::unique(fr.begin(), fr.end()), fr.end());
    } else {
        fr = columnLateralBraceHeightFractionsEffective(input);
    }
    if (fr.empty()) {
        return Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iz_m4, input.columnBucklingKz, H);
    }
    std::vector<double> pts;
    pts.push_back(0.0);
    for (const double f : fr) {
        pts.push_back(f);
    }
    pts.push_back(1.0);
    double ncrMin = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double L = (pts[i + 1] - pts[i]) * H;
        if (L < 1e-9) {
            continue;
        }
        ncrMin = std::min(ncrMin, Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iz_m4, 1.0, L));
    }
    if (ncrMin >= std::numeric_limits<double>::max() / 2.0) {
        return Ec3Buckling::eulerAxialCriticalForce_N(E_Pa, Iz_m4, input.columnBucklingKz, H);
    }
    return ncrMin;
}

double PortalSolver::trussSelfWeightHoriz_kN_per_m2(const PortalFrameInput &input)
{
    PortalFrameResult geom;
    buildPortalGeometry(input, geom);
    if (input.trussMemberSectionForm == 3) {
        const double Ha = trussApexHeight_m(input);
        const double Hc = input.columnHeight_m;
        const double W = input.spanWidth_m;
        const double halfLen = std::hypot(0.5 * W, std::max(Ha - Hc, 1e-12));
        const double trussLen_m = 2.0 * halfLen;
        constexpr double gammaSteel_kN_per_m3 = 77.0;
        const double weight_kN = gammaSteel_kN_per_m3 * input.steelArea_m2 * trussLen_m;
        const double footprint_m2 =
            std::max(W, 1e-12) * std::max(PortalSolver::trussSteelDeadTributaryReferenceY_m, 1e-12);
        return weight_kN / footprint_m2;
    }
    double trussLen_m = 0.0;
    auto nodeXY = [&](int tag, double &ox, double &oy) -> bool {
        for (const auto &n : geom.nodes) {
            if (n.tag == tag) {
                ox = n.x;
                oy = n.y;
                return true;
            }
        }
        return false;
    };
    for (const auto &m : geom.members) {
        if (!m.isTruss) {
            continue;
        }
        double x1 = 0.0;
        double y1 = 0.0;
        double x2 = 0.0;
        double y2 = 0.0;
        if (!nodeXY(m.nodeI, x1, y1) || !nodeXY(m.nodeJ, x2, y2)) {
            continue;
        }
        trussLen_m += std::hypot(x2 - x1, y2 - y1);
    }
    constexpr double gammaSteel_kN_per_m3 = 77.0;
    const double weight_kN = gammaSteel_kN_per_m3 * input.steelArea_m2 * trussLen_m;
    const double W = input.spanWidth_m;
    const double footprint_m2 =
        std::max(W, 1e-12) * std::max(PortalSolver::trussSteelDeadTributaryReferenceY_m, 1e-12);
    return weight_kN / footprint_m2;
}

double PortalSolver::roofPurlinDeadSurface_kN_per_m2()
{
    constexpr double kAsikDead_kg_per_horizontal_m2 = 7.0;
    constexpr double kGrav = 9.80665;
    return kAsikDead_kg_per_horizontal_m2 * kGrav / 1000.0;
}

TurkishLoads::CharacteristicLineLoadsPerM PortalFrameInput::lineLoadsCharacteristic() const
{
    const double S = std::max(trussAxisSpacingY_m, 1e-12);
    TurkishLoads::CharacteristicLineLoadsPerM c;
    /** Makas çeliği kN/m² tabanı W·Yref; ×S ile çelik çizgi yükü S ile orantılı (tüm dl1 bileşenleri Y’ye bağlı). */
    const double trussHoriz = PortalSolver::trussSelfWeightHoriz_kN_per_m2(*this);
    const double asikSurface_kN_per_m2 = PortalSolver::roofPurlinDeadSurface_kN_per_m2();
    c.dl1 = (trussHoriz + asikSurface_kN_per_m2 + purlinCladding_kN_per_m2) * S;
    c.dl2 = dl2_kN_per_m2 * S;
    constexpr double kSnowRoofShapeMu = 0.8;
    c.sn = sk_kN_per_m2 * kSnowRoofShapeMu * S;
    c.wl = wl_kN_per_m2 * S;
    return c;
}
