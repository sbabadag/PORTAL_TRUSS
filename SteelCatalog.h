#pragma once

#include <optional>
#include <QString>
#include <vector>

/** Eurocode-style rolled I-sections (HEA / HEB / IPE); properties from standard tables (design aid). */
struct RolledISection
{
    QString designation; ///< e.g. "HEA 240"
    double A_m2{0.0};    ///< gross area
    double Wy_m3{0.0};   ///< elastic section modulus y-y (güçlü eksen) — 2B portal kolon eğilmesi (web düzlemde)
    double Wz_m3{0.0};   ///< elastic section modulus z-z (zayıf eksen)
    double Iy_m4{0.0};   ///< y-y (güçlü eksen) atalet — kolon burkulması
    double Iz_m4{0.0};   ///< z-z (zayıf eksen) atalet — kolon burkulması
    double mass_kg_per_m{0.0};
    /** EN 10365 tipik boyutlar [mm] — sınıflandırma / It,Iw; 0 = bilinmiyor. */
    double h_mm{0.0};
    double b_mm{0.0};
    double tw_mm{0.0};
    double tf_mm{0.0};
    double r_mm{0.0};
    /** Saint-Venant torsiyon sabiti ve burulma sabiti (hadde I yaklaşımı). */
    double It_m4{0.0};
    double Iw_m6{0.0};
};

/** Back-to-back equal-leg angles 2xL (same section, gap neglected for area). */
struct DoubleAngleSection
{
    QString designation; ///< e.g. "2xL 80x80x8"
    double A_total_m2{0.0}; ///< 2 * A_single
    /** EN 10056 eşit bacak tek açı — zayıf eksen iv [m]. */
    double i_min_m{0.0};
    /** 2×L bilesik: sqrt(iv^2+(d/2)^2) [m]; d yaklasik sirt boslugu + kalinlik. */
    double i_buckling_m{0.0};
    double mass_kg_per_m{0.0};
};

enum class ColumnFamily : int
{
    Hea = 0,
    Heb = 1,
    Ipe = 2
};

namespace SteelCatalog
{

const std::vector<RolledISection> &heaSections();
const std::vector<RolledISection> &hebSections();
const std::vector<RolledISection> &ipeSections();

const std::vector<RolledISection> &sectionsForFamily(ColumnFamily f);

const std::vector<DoubleAngleSection> &doubleAngles2L();

/** Kesit adı katalogda varsa (örn. "2xL 80x80x8"); aşım/trim sonrası eşleşmez. */
std::optional<DoubleAngleSection> tryGetDoubleAngle2L(QString designation);

/** HEA / HEB / IPE kataloglarında tam ad (örn. "IPE 200"); aşım eki temizlenir. */
std::optional<RolledISection> tryGetRolledI(QString designation);

} // namespace SteelCatalog
