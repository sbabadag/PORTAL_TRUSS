#pragma once

#include <algorithm>
#include <QString>
#include <QMetaType>
#include <vector>

#include "SectionOptimizer.h"
#include "TurkishLoads.h"

/**
 * Input for a symmetric steel portal frame with A-truss rafters.
 * Geometry: m; E Pa; I m⁴; A m².
 * Çatı/kolon karakteristik yükleri kN/m² (yoğunluk); makas aks aralığı Y ile çarpılarak 2B çerçeveye kN/m yayılı yük verilir.
 */
struct PortalFrameInput
{
    double spanWidth_m{12.0};
    double columnHeight_m{6.0};
    /** Makas tepesi [m]; `trussApexHeight_m` ile doldurulur (L = tam açıklık W: kolon yüzünde L/40, ortada L/11). */
    double apexHeight_m{8.0};
    int trussPanelsPerSide{4};

    /** Plan Y yönünde yan yana makas/çerçeve aksları arası mesafe [m] — tributary genişlik. */
    double trussAxisSpacingY_m{5.0};

    double steelArea_m2{0.015};
    double steelInertia_m4{5.0e-4};
    double youngModulus_Pa{200e9};

    /** Aşık + çatı kaplaması — kN/m² (yatay izdüşüm); makas çeliği ayrıca otomatik eklenir. */
    double purlinCladding_kN_per_m2{0.15};
    /** Çatı ölü 2 — kN/m². */
    double dl2_kN_per_m2{0.2};
    /** Kar düzlem yükü SK — kN/m² (kullanıcı veri girişi, TS 498 / harita); çatı şekil katsayısı μ=0.8 ile çarpılır. */
    double sk_kN_per_m2{0.375};
    /** Rüzgar eşdeğeri — kN/m² kolon yüzeyine; hat yükü × aks aralığı. */
    double wl_kN_per_m2{0.16};

    /** 0 = HEA, 1 = HEB, 2 = IPE (kolon için en hafif uygun profil seçilir). */
    int columnFamilyIndex{0};
    /** Çelik akma dayanımı (S235 → 235 MPa). */
    double fy_MPa{235.0};

    TurkishLoads::CharacteristicLineLoadsPerM lineLoadsCharacteristic() const;
};

struct NodeResult
{
    int tag{0};
    double x{0.0};
    double y{0.0};
    double ux{0.0};
    double uy{0.0};
    double rz{0.0};
};

/** Makas çubuğu rolü — çizimde kalınlık / etiket; analizde tüm makas aynı A,I. */
enum class TrussMemberRole : uint8_t
{
    Column,
    ChordBottom,
    ChordTop,
    EdgePost,
    Web,
    /** Sol/sağ uçta kısa düşey gusset (alt hat köşesinden aşağı; kesit alt hat 2×L ile aynı grup). */
    GussetStrip
};

struct MemberResult
{
    int tag{0};
    int nodeI{0};
    int nodeJ{0};
    /** Axial force (N) for truss; for beam columns, first value used as representative. */
    double axial_N{0.0};
    double moment_i_Nm{0.0};
    double moment_j_Nm{0.0};
    bool isTruss{true};
    /** Üst çatı hattı (aşık): OpenSees’te yalnızca bu çubuklara yatay izdüşümle çatı yayılı yükü. */
    bool carriesRoofLineLoad{false};
    TrussMemberRole trussRole{TrussMemberRole::Column};
    /** İlk dört iç üst hat paneli için 1..4; çizim etiketi; 0 = etiket yok. */
    uint8_t topChordLabelNumber{0};
    bool topChordLabelOnLeft{true};
    /**
     * Köşegen sanal bölgesi (global sıra w: her j için önce sol, sonra sağ; w = 0…2N−1).
     * 0 = üst hat ile aynı 2×L (ilk 2 çubuk); 1 = B (60×60×6’dan arama); 2 = C (50×50×5);
     * 3 = sondan bir önceki çubuk (60×60×6’dan arama). Kolon/üst/alt çubuklarda 0.
     */
    uint8_t webOptZone{0};
};

struct PortalFrameResult
{
    bool ok{false};
    QString errorMessage;

    std::vector<NodeResult> nodes;
    std::vector<MemberResult> members;

    SectionOptimizationResult sectionDesign;
};

Q_DECLARE_METATYPE(PortalFrameResult)

/**
 * Geometri + OpenSees Tcl statik analizi (OpenSees.exe / OPENSEES_EXE).
 * İşçi iş parçacığında çağrılabilir; QWidget kullanmayın.
 */
class PortalSolver
{
public:
    PortalSolver() = default;

    /** Runs the full analysis (blocking). */
    PortalFrameResult analyze(const PortalFrameInput &input) const;

    /** Tepe düğümü yüksekliği: L = tam açıklık W → kolonda makas derinliği L/40, ortada L/11 (düz üst hat). */
    static double trussApexHeight_m(const PortalFrameInput &input);

    /** Fills nodes/members for visualization and for mirroring OpenSees connectivity. */
    static void buildPortalGeometry(const PortalFrameInput &input, PortalFrameResult &out);

    /** Makas çubukları toplam çelik hacmi → yatay izdüşüm m² başına kN/m² (γ≈77 kN/m³). */
    static double trussSelfWeightHoriz_kN_per_m2(const PortalFrameInput &input);

private:
    bool validate(const PortalFrameInput &in, QString *err) const;
};
