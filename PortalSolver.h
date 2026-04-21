#pragma once

#include <algorithm>
#include <QString>
#include <QMetaType>
#include <vector>

#include "SectionOptimizer.h"
#include "TurkishLoads.h"

/** Kolon tabanı: OpenSees’te düğüm rz tutuluğu (ankastre) veya serbest (mafsallı). */
enum class ColumnBaseSupport : int
{
    Fixed = 0,
    Pinned = 1
};

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

    /**
     * Kolon eksenel burkulma (basit Euler + χ): y-y için effektif boy katsayısı Ky (genelde salınım ~2).
     * Şartname / bağlantı ile doğrulayın.
     */
    double columnBucklingKy{2.0};
    /**
     * z-z burkulma: tutuluş yokken Kz (varsayılan 2); ara yanal tutuluş verilirse segmentlerde K≈1 kullanılır.
     */
    double columnBucklingKz{2.0};
    /** Kolon eksenel χ imperfection: 0=A0, 1=A, 2=B, 3=C, 4=D (EN 1993-1-1 Tablo 6.1). */
    int columnBucklingCurveOrdinal{2};
    /** Makas 2×L eksenel χ için aynı kodlama. */
    int trussBucklingCurveOrdinal{2};
    /**
     * Tabandan kolon boyuna oran (0,1) içinde yanal tutuluş yükseklikleri; virgülle ayrılmış girişten doldurulur.
     * `columnZzBucklingBraceHeightFractions` boşsa z-z Euler segmentleri bununla (+ isteğe bağlı makas türetilmiş) birleştirilir.
     */
    std::vector<double> columnLateralBraceHeightFractions;
    /**
     * Z-z eksenel burkulma (Euler Iz) için tutuluş kesitleri — yalnızca bu oranlar kullanılır; elle/makas listesi ile karıştırılmaz.
     * Boşsa `columnLateralBraceHeightFractions` + `columnLateralBraceFromTrussGeometry` etkisi devam eder.
     */
    std::vector<double> columnZzBucklingBraceHeightFractions;
    /**
     * true: z-z burkulmada `columnTrussDerivedBraceHeightFractions` ile makas alt hattı (+ varsa 45° köşe gusset) kolon
     * düğüm yükseklikleri otomatik eklenir (elle girilen oranlarla birleştirilir). Üst hat bu şemada kolona
     * yalnızca mahyada (y=Hc) bağlanır — uç sınır; ara h/H olarak eklenmez.
     */
    bool columnLateralBraceFromTrussGeometry{true};

    /**
     * LTB: alt başlık (compression flange tarafı) yanal tutuluş h/H — segment bazlı Mcr, χ_LT.
     * İki flanş listesi de boşsa LTB uygulanmaz.
     */
    std::vector<double> columnLtbBottomFlangeBraceHeightFractions;
    /** LTB: üst başlık yanal tutuluş h/H. Her iki flanş da doluysa Mcr = min(Mcr,alt, Mcr,üst) (muhafazakâr). */
    std::vector<double> columnLtbTopFlangeBraceHeightFractions;
    /** LTB imperfection curve ordinal: 0…4 = A0…D (EN 1993-1-1 Tablo 6.3 için pratik eşleme; varsayılan B). */
    int columnLtbCurveOrdinal{2};

    /** Kolon ayak mesneti — statik model (OpenSees) ve el kolon momenti kabulleri. */
    ColumnBaseSupport columnBaseSupport{ColumnBaseSupport::Fixed};

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
    /** Sol/sağ uçta 45° köşe gusset (kolon üzerinde alt hattan aşağı → alt başlık köşesine; kesit alt hat 2×L ile aynı grup). */
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

    /** OpenSees sonrası: pin-eksen makas modeli ile makas eksenel kuvvet farkı özeti (boş olabilir). */
    QString lectureTrussComparisonNote;
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

    /**
     * Kolon z-z burkulması için makas geometrisinden h/H oranları — `buildPortalGeometry` ile uyumlu.
     * Alt hat köşesi y = Hc − W/40; gusset varsa kolon üzerindeki alt uç y (45° bacak, tercihen 1 m).
     */
    static std::vector<double> columnTrussDerivedBraceHeightFractions(const PortalFrameInput &input);

    /** Elle girilen + (isteğe bağlı) makastan türetilen tutuluş oranlarının birleşimi (sıralı, tekil). */
    static std::vector<double> columnLateralBraceHeightFractionsEffective(const PortalFrameInput &input);

    /** z-z Euler kritik kuvvet Ncr [N] — `columnLateralBraceHeightFractionsEffective` ile segmentli veya Kz·H. */
    static double columnElasticCriticalForceZz_N(double E_Pa, double Iz_m4, const PortalFrameInput &input);

    /** Makas çubukları toplam çelik hacmi → yatay izdüşüm m² başına kN/m² (γ≈77 kN/m³). */
    static double trussSelfWeightHoriz_kN_per_m2(const PortalFrameInput &input);

    /** Giriş tutarlılığı (analyze / self-check öncesi). */
    bool validate(const PortalFrameInput &in, QString *err) const;
};
