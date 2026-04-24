#pragma once

#include <QString>
#include <vector>

/**
 * TS 498 "Yapılara Yükler Standardı" — characteristic line loads (no point loads).
 *
 * Roof (çatı / makas üst hat):
 *   Ölü 1 hat yükü (dl1): makas çeliği (otom., W·Yref tabanı) + aşık 7 kg/m² + ek ölü (kN/m²) — hepsi × Y → kN/m.
 *   DL2 — kN/m²; × Y aks aralığı.
 *   Kar: SK (kN/m²) × çatı şekil katsayısı μ=0,8 × Y → hat üzerinde kar yayılısı sn (kN/m).
 *
 * Wind (rüzgar):
 *   WL — kN/m² kolon yüzeyine eşdeğer; × aks aralığı Y = kolon boyuna kN/m.
 *
 * STR (deprem dışı) kısmi katsayıları: proje şartnamesi / TS 498 baskısı ile doğrulayın.
 */
namespace TurkishLoads {

/** Characteristic uniform line loads. */
struct CharacteristicLineLoadsPerM
{
    /** Roof: dead line load 1 — makas çeliği (W·Yref tabanı×Y) + aşık 7 kg/m² + ek ölü, hepsi ×Y (kN/m). */
    double dl1{0.0};
    /** Roof: dead component 2 (kN/m). */
    double dl2{0.0};
    /** Roof: snow line load (kN/m along roof member). */
    double sn{0.0};
    /** Column: wind horizontal line load (kN/m of column height). */
    double wl{0.0};
};

struct Ts498PartialFactors
{
    double gammaG{1.35};
    double gammaSnow{1.50};
    double gammaWind{1.50};
    double psi0SnowWithWind{0.50};
};

/**
 * Deprem dışı STR (TS 498 + TS EN 1990/1991 ile uyumlu) tipik kombinasyonlar — çatı hat yükü ve kol rüzgârı tasarım değerleri.
 * Her biri için ayrı elastik analiz yapılıp kesit zarfı alınır.
 */
struct StrCombination
{
    QString id; ///< kısa kod (örn. STR-1)
    QString description;
    double q_roof_design_kN_m{0.0};   ///< çatı/makas hattı tasarım yayılı yükü (kN/m)
    double w_column_design_kN_m{0.0}; ///< kolon yanal tasarım yayılı yükü (kN/m)
};

/** Kar baskın, rüzgâr baskın (eşlik kar), ölü+rüzgar (kar yok) kombinasyonları. */
std::vector<StrCombination> strCombinationsDepremDisi(const CharacteristicLineLoadsPerM &c,
                                                      const Ts498PartialFactors &f = {});

/** Roof: dominant snow — qd = 1.35(DL1+DL2) + 1.50·SN (kN/m). */
double roofLineLoadDesignSnowDominant_kN_per_m(const CharacteristicLineLoadsPerM &c,
                                               const Ts498PartialFactors &f = {});

/** Roof: wind leading — accompanying snow reduced: 1.35(DL1+DL2) + 1.50·ψ0·SN (kN/m). */
double roofLineLoadDesignWindLeading_kN_per_m(const CharacteristicLineLoadsPerM &c,
                                              const Ts498PartialFactors &f = {});

/** Column: design wind line load wd = 1.50·WL (kN/m). */
double columnWindLineLoadDesign_kN_per_m(const CharacteristicLineLoadsPerM &c,
                                         const Ts498PartialFactors &f = {});

QString formatRegulatorySummary(const CharacteristicLineLoadsPerM &c, const Ts498PartialFactors &f = {});

} // namespace TurkishLoads
