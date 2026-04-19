#include "PortalSolver.h"
#include "OpenSeesRunner.h"
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

} // namespace

double PortalSolver::trussApexHeight_m(const PortalFrameInput &input)
{
    const double L = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    const double hEdge = L / 40.0;
    return (Hc - hEdge) + L / 11.0;
}

bool PortalSolver::validate(const PortalFrameInput &in, QString *err) const
{
    if (in.spanWidth_m <= 0.0 || in.columnHeight_m <= 0.0) {
        if (err) {
            *err = QStringLiteral("Span and column height must be positive.");
        }
        return false;
    }
    if (in.columnHeight_m <= in.spanWidth_m / 40.0 + 1e-12) {
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
        + QStringLiteral(" OpenSees STR zarf: %1. %2 Kolon: %3, makas: %4.")
              .arg(combIds)
              .arg(out.sectionDesign.envelopeNote)
              .arg(out.sectionDesign.columnProfile)
              .arg(out.sectionDesign.trussProfile2xL);
    return out;
}

void PortalSolver::buildPortalGeometry(const PortalFrameInput &input, PortalFrameResult &out)
{
    const double W = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    const double Lhalf = 0.5 * W;
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

    const int idLeftBase = addNode(0.0, 0.0);
    const int idRightBase = addNode(W, 0.0);
    /** Üst hat kolon tepesi (Hc); alt hat tam açıklık L=W için L/40 aşağıda; köşede düşey çubuk yok (ilk çapraz alttan). */
    const int idLeftEave = addNode(0.0, Hc);
    const int idRightEave = addNode(W, Hc);
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

    /** Yalnızca sol/sağ alt köşede kısa düşey gusset — ilk çaprazı statik olarak rahatlatmak için (şema ucu). */
    const double yBotCorner = Hc - hEdge;
    const double stubDrop =
        std::clamp(W / 120.0, 0.03, std::max(0.02, std::min(0.12 * Hc, yBotCorner - 0.02)));
    const double yGusEnd = std::max(yBotCorner - stubDrop, 1e-3);
    const int idLeftGusEnd = addNode(0.0, yGusEnd);
    const int idRightGusEnd = addNode(W, yGusEnd);

    auto addMember = [&](int i, int j, bool truss, bool roofLine, TrussMemberRole role, uint8_t tcLbl = 0,
                         bool tcLeft = true, uint8_t webOptZone = 0) {
        MemberResult m;
        m.tag = static_cast<int>(out.members.size()) + 1;
        m.nodeI = i;
        m.nodeJ = j;
        m.isTruss = truss;
        m.carriesRoofLineLoad = roofLine;
        m.trussRole = truss ? role : TrussMemberRole::Column;
        m.topChordLabelNumber = tcLbl;
        m.topChordLabelOnLeft = tcLeft;
        m.webOptZone = (truss && role == TrussMemberRole::Web) ? webOptZone : 0;
        out.members.push_back(m);
    };

    addMember(idLeftBase, idLeftEave, false, false, TrussMemberRole::Column);
    addMember(idRightBase, idRightEave, false, false, TrussMemberRole::Column);

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

    if (stubDrop > 1e-6 && yGusEnd + 1e-9 < yBotCorner) {
        addMember(idLeftBottom, idLeftGusEnd, true, false, TrussMemberRole::GussetStrip);
        addMember(idRightBottom, idRightGusEnd, true, false, TrussMemberRole::GussetStrip);
    }

    const int wtot = 2 * N;
    int webSeq = 0;
    for (int j = 0; j < N; ++j) {
        const bool zig = (j % 2) == 0;
        const uint8_t zL = webOptZoneFromGlobalSeq(webSeq++, wtot);
        const uint8_t zR = webOptZoneFromGlobalSeq(webSeq++, wtot);
        if (zig) {
            addMember(bottomLeft[static_cast<size_t>(j)], topLeft[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zL);
            addMember(bottomRight[static_cast<size_t>(j)], topRight[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zR);
        } else {
            addMember(topLeft[static_cast<size_t>(j)], bottomLeft[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zL);
            addMember(topRight[static_cast<size_t>(j)], bottomRight[static_cast<size_t>(j + 1)], true, false,
                      TrussMemberRole::Web, 0, true, zR);
        }
    }
}

double PortalSolver::trussSelfWeightHoriz_kN_per_m2(const PortalFrameInput &input)
{
    PortalFrameResult geom;
    buildPortalGeometry(input, geom);
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
    const double footprint_m2 =
        std::max(input.spanWidth_m, 1e-12) * std::max(input.trussAxisSpacingY_m, 1e-12);
    return weight_kN / footprint_m2;
}

TurkishLoads::CharacteristicLineLoadsPerM PortalFrameInput::lineLoadsCharacteristic() const
{
    const double S = std::max(trussAxisSpacingY_m, 1e-12);
    TurkishLoads::CharacteristicLineLoadsPerM c;
    const double trussHoriz = PortalSolver::trussSelfWeightHoriz_kN_per_m2(*this);
    c.dl1 = (trussHoriz + purlinCladding_kN_per_m2) * S;
    c.dl2 = dl2_kN_per_m2 * S;
    constexpr double kSnowRoofShapeMu = 0.8;
    c.sn = sk_kN_per_m2 * kSnowRoofShapeMu * S;
    c.wl = wl_kN_per_m2 * S;
    return c;
}
