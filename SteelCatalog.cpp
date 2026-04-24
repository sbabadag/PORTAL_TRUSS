#include "SteelCatalog.h"

#include <algorithm>
#include <cmath>

namespace {

/** A [mm²], Wy/Wz [cm³], Iy/Iz [cm⁴], m [kg/m]; h,b,tw,tf,r [mm] — EN 10365 tipik. */
RolledISection makeI(const char *name, double A_mm2, double Wy_cm3, double Iy_cm4, double Iz_cm4, double Wz_cm3,
                     double m, double h_mm, double b_mm, double tw_mm, double tf_mm, double r_mm)
{
    RolledISection s;
    s.designation = QString::fromUtf8(name);
    s.A_m2 = A_mm2 * 1e-6;
    s.Wy_m3 = Wy_cm3 * 1e-6;
    s.Wz_m3 = Wz_cm3 * 1e-6;
    s.Iy_m4 = Iy_cm4 * 1e-8;
    s.Iz_m4 = Iz_cm4 * 1e-8;
    s.mass_kg_per_m = m;
    s.h_mm = h_mm;
    s.b_mm = b_mm;
    s.tw_mm = tw_mm;
    s.tf_mm = tf_mm;
    s.r_mm = r_mm;
    if (h_mm > 1.0 && b_mm > 1.0 && tw_mm > 0.0 && tf_mm > 0.0) {
        const double rUse = (r_mm > 0.5) ? r_mm : std::max(1.5 * tw_mm, 8.0);
        const double hw_mm = std::max(h_mm - 2.0 * tf_mm - 2.0 * rUse, 0.0);
        const double hwRect_mm = std::max(h_mm - 2.0 * tf_mm, 0.0);
        const double yFlange_mm = 0.5 * (h_mm - tf_mm);
        const double IyGeom_mm4 =
            2.0 * (b_mm * tf_mm * tf_mm * tf_mm / 12.0 + b_mm * tf_mm * yFlange_mm * yFlange_mm)
            + tw_mm * hwRect_mm * hwRect_mm * hwRect_mm / 12.0;
        const double IzGeom_mm4 = 2.0 * (tf_mm * b_mm * b_mm * b_mm / 12.0)
                                + hwRect_mm * tw_mm * tw_mm * tw_mm / 12.0;
        if ((Iy_cm4 <= 0.0 || Iz_cm4 <= 0.0 || Wy_cm3 <= 0.0 || Wz_cm3 <= 0.0) && IyGeom_mm4 > 1e-6
            && IzGeom_mm4 > 1e-6) {
            /** Fallback only: use geometry-derived properties when the table row has missing section properties. */
            s.Iy_m4 = IyGeom_mm4 * 1e-12;
            s.Iz_m4 = IzGeom_mm4 * 1e-12;
            s.Wy_m3 = s.Iy_m4 / std::max(0.5 * h_mm * 1e-3, 1e-9);
            s.Wz_m3 = s.Iz_m4 / std::max(0.5 * b_mm * 1e-3, 1e-9);
        }
        /** It ≈ Σ bt³/3 (ince dikdörtgen); Iw ≈ Iz·h₀²/4, h₀ = flans ağırlık merkezleri arası [m]. */
        s.It_m4 = (2.0 * b_mm * tf_mm * tf_mm * tf_mm / 3.0 + hw_mm * tw_mm * tw_mm * tw_mm / 3.0) * 1e-12;
        const double h0_m = std::max((h_mm - tf_mm) * 1e-3, 1e-6);
        s.Iw_m6 = s.Iz_m4 * h0_m * h0_m / 4.0;
    }
    return s;
}

/**
 * iv_mm: EN 10056 eşit bacak tek açı — asgari atalet yarıçapı (zayıf eksen), mm.
 * Makas 2×L: bileşik burkulma için iki açı ağırlık merkezleri arası yaklaşık d ≈ tipik sırt boşluğu + kalınlık.
 */
DoubleAngleSection make2L(const char *name, double leg_mm, double thk_mm, double A_one_mm2, double m2, double iv_mm)
{
    DoubleAngleSection d;
    d.designation = QString::fromUtf8(name);
    d.A_total_m2 = 2.0 * A_one_mm2 * 1e-6;
    d.mass_kg_per_m = m2;
    d.i_min_m = std::max(iv_mm * 1e-3, 0.006);
    constexpr double kTypicalGapMm = 12.0;
    const double d_half_mm = 0.5 * (kTypicalGapMm + thk_mm);
    d.i_buckling_m = std::sqrt(iv_mm * iv_mm + d_half_mm * d_half_mm) * 1e-3;
    (void)leg_mm;
    return d;
}

} // namespace

namespace SteelCatalog {

static std::vector<RolledISection> g_hea;
static std::vector<RolledISection> g_heb;
static std::vector<RolledISection> g_ipe;
static std::vector<DoubleAngleSection> g_2l;

static void initIfNeeded()
{
    if (!g_hea.empty()) {
        return;
    }

    // HEA — values imported from Profil Tablo.xls / HE sheet.
    g_hea = {
        makeI("HEA 100", 2124, 72.76, 349.2, 133.8, 26.76, 16.7, 96, 100, 5, 8, 12),
        makeI("HEA 120", 2534, 106.3, 606.2, 230.9, 38.48, 19.9, 114, 120, 5, 8, 12),
        makeI("HEA 140", 3142, 155.4, 1033, 389.3, 55.62, 24.7, 133, 140, 5.5, 8.5, 12),
        makeI("HEA 160", 3877, 220.1, 1673, 615.6, 76.95, 30.4, 152, 160, 6, 9, 15),
        makeI("HEA 180", 4525, 293.6, 2510, 924.6, 102.7, 35.5, 171, 180, 6, 9.5, 15),
        makeI("HEA 200", 5383, 388.6, 3692, 1336, 133.6, 42.3, 190, 200, 6.5, 10, 18),
        makeI("HEA 220", 6434, 515.2, 5410, 1955, 177.7, 50.5, 210, 220, 7, 11, 18),
        makeI("HEA 240", 7684, 675.1, 7763, 2769, 230.7, 60.3, 230, 240, 7.5, 12, 21),
        makeI("HEA 260", 8682, 836.4, 10450, 3668, 282.1, 68.2, 250, 260, 7.5, 12.5, 24),
        makeI("HEA 280", 9726, 1013, 13670, 4763, 340.2, 76.4, 270, 280, 8, 13, 24),
        makeI("HEA 300", 11250, 1260, 18260, 6310, 420.6, 88.3, 290, 300, 8.5, 14, 27),
        makeI("HEA 320", 12440, 1479, 22930, 6985, 465.7, 97.6, 310, 300, 9, 15.5, 27),
        makeI("HEA 340", 13350, 1678, 27690, 7436, 495.7, 105, 330, 300, 9.5, 16.5, 27),
        makeI("HEA 360", 14280, 1891, 33090, 7887, 525.8, 112, 350, 300, 10, 17.5, 27),
        makeI("HEA 400", 15900, 2311, 45070, 8564, 570.9, 125, 390, 300, 11, 19, 27),
        makeI("HEA 450", 17800, 2896, 63720, 9465, 631, 140, 440, 300, 11.5, 21, 27),
        makeI("HEA 500", 19750, 3550, 86970, 10370, 691.1, 155, 490, 300, 12, 23, 27),
        makeI("HEA 550", 21180, 4146, 111900, 10820, 721.3, 166, 540, 300, 12.5, 24, 27),
        makeI("HEA 600", 22650, 4787, 141200, 11270, 751.4, 178, 590, 300, 13, 25, 27),
    };

    // HEB — values imported from Profil Tablo.xls / HE sheet.
    g_heb = {
        makeI("HEB 100", 2604, 89.91, 449.5, 167.3, 33.45, 20.4, 100, 100, 6, 10, 12),
        makeI("HEB 120", 3401, 144.1, 864.4, 317.5, 52.92, 26.7, 120, 120, 6.5, 11, 12),
        makeI("HEB 140", 4296, 215.6, 1509, 549.7, 78.52, 33.7, 140, 140, 7, 12, 12),
        makeI("HEB 160", 5425, 311.5, 2492, 889.2, 111.2, 42.6, 160, 160, 8, 13, 15),
        makeI("HEB 180", 6525, 425.7, 3831, 1363, 151.4, 51.2, 180, 180, 8.5, 14, 15),
        makeI("HEB 200", 7808, 569.6, 5696, 2003, 200.3, 61.3, 200, 200, 9, 15, 18),
        makeI("HEB 220", 9104, 735.5, 8091, 2843, 258.5, 71.5, 220, 220, 9.5, 16, 18),
        makeI("HEB 240", 10600, 938.3, 11260, 3923, 326.9, 83.2, 240, 240, 10, 17, 21),
        makeI("HEB 260", 11840, 1148, 14920, 5135, 395, 93, 260, 260, 10, 17.5, 24),
        makeI("HEB 280", 13140, 1376, 19270, 6595, 471, 103, 280, 280, 10.5, 18, 24),
        makeI("HEB 300", 14910, 1678, 25170, 8563, 570.9, 117, 300, 300, 11, 19, 27),
        makeI("HEB 320", 16130, 1926, 30820, 9239, 615.9, 127, 320, 300, 11.5, 20.5, 27),
        makeI("HEB 340", 17090, 2156, 36660, 9690, 646, 134, 340, 300, 12, 21.5, 27),
        makeI("HEB 360", 18060, 2400, 43190, 10140, 676.1, 142, 360, 300, 12.5, 22.5, 27),
        makeI("HEB 400", 19780, 2884, 57680, 10820, 721.3, 155, 400, 300, 13.5, 24, 27),
        makeI("HEB 450", 21800, 3551, 79890, 11720, 781.4, 171, 450, 300, 14, 26, 27),
        makeI("HEB 500", 23860, 4287, 107200, 12620, 841.6, 187, 500, 300, 14.5, 28, 27),
        makeI("HEB 550", 25410, 4971, 136700, 13080, 871.8, 199, 550, 300, 15, 29, 27),
        makeI("HEB 600", 27000, 5701, 171000, 13530, 902, 212, 600, 300, 15.5, 30, 27),
    };

    // IPE — values imported from Profil Tablo.xls / IPE sheet.
    g_ipe = {
        makeI("IPE 100", 1032, 34.2, 171, 15.92, 5.79, 8.1, 100, 55, 4.1, 5.7, 7),
        makeI("IPE 120", 1321, 52.96, 317.8, 27.67, 8.65, 10.4, 120, 64, 4.4, 6.3, 7),
        makeI("IPE 140", 1643, 77.32, 541.2, 44.92, 12.31, 12.9, 140, 73, 4.7, 6.9, 7),
        makeI("IPE 160", 2009, 108.7, 869.3, 68.31, 16.66, 15.8, 160, 82, 5, 7.4, 9),
        makeI("IPE 180", 2395, 146.3, 1317, 100.9, 22.16, 18.8, 180, 91, 5.3, 8, 9),
        makeI("IPE 200", 2848, 194.3, 1943, 142.4, 28.47, 22.4, 200, 100, 5.6, 8.5, 12),
        makeI("IPE 220", 3337, 252, 2772, 204.9, 37.25, 26.2, 220, 110, 5.9, 9.2, 12),
        makeI("IPE 240", 3912, 324.3, 3892, 283.6, 47.27, 30.7, 240, 120, 6.2, 9.8, 15),
        makeI("IPE 270", 4595, 428.9, 5790, 419.9, 62.2, 36.1, 270, 135, 6.6, 10.2, 15),
        makeI("IPE 300", 5381, 557.1, 8356, 603.8, 80.5, 42.2, 300, 150, 7.1, 10.7, 15),
        makeI("IPE 330", 6261, 713.1, 11770, 788.1, 98.52, 49.1, 330, 160, 7.5, 11.5, 18),
        makeI("IPE 360", 7273, 903.6, 16270, 1043, 122.8, 57.1, 360, 170, 8, 12.7, 18),
        makeI("IPE 400", 8446, 1156, 23130, 1318, 146.4, 66.3, 400, 180, 8.6, 13.5, 21),
        makeI("IPE 450", 9882, 1500, 33740, 1676, 176.4, 77.6, 450, 190, 9.4, 14.6, 21),
        makeI("IPE 500", 11550, 1928, 48200, 2142, 214.2, 90.7, 500, 200, 10.2, 16, 21),
        makeI("IPE 600", 15600, 3069, 92080, 3387, 307.9, 122, 600, 220, 12, 19, 24),
    };

    // iv [mm] — EN 10056 eşit bacak, asgari atalet yarıçapı (zayıf eksen).
    g_2l = {
        make2L("2xL 45x45x4", 45, 4, 349, 4.2, 10.9),
        make2L("2xL 50x50x5", 50, 5, 480, 5.8, 12.2),
        make2L("2xL 60x60x6", 60, 6, 691, 8.4, 14.6),
        make2L("2xL 70x70x7", 70, 7, 940, 11.4, 16.9),
        make2L("2xL 80x80x8", 80, 8, 1220, 14.8, 15.5),
        make2L("2xL 90x90x9", 90, 9, 1550, 18.8, 17.4),
        make2L("2xL 100x100x8", 100, 8, 1551, 24.4, 19.6),
        make2L("2xL 100x100x10", 100, 10, 1920, 30.0, 19.5),
        make2L("2xL 100x100x12", 100, 12, 2271, 35.6, 19.4),
        make2L("2xL 110x110x10", 110, 10, 2118, 33.2, 21.5),
        make2L("2xL 110x110x12", 110, 12, 2514, 39.4, 21.4),
        make2L("2xL 120x120x10", 120, 10, 2318, 36.4, 23.5),
        make2L("2xL 120x120x11", 120, 11, 2537, 39.8, 23.5),
        make2L("2xL 120x120x12", 120, 12, 2754, 43.2, 23.4),
        make2L("2xL 120x120x13", 120, 13, 2969, 46.6, 23.4),
        make2L("2xL 120x120x15", 120, 15, 3393, 53.2, 23.3),
        make2L("2xL 130x130x12", 130, 12, 3000, 47.0, 25.4),
        make2L("2xL 140x140x10", 140, 10, 2724, 42.8, 27.6),
        make2L("2xL 140x140x13", 140, 13, 3495, 54.8, 27.4),
        make2L("2xL 150x150x10", 150, 10, 2927, 46.0, 29.6),
        make2L("2xL 150x150x12", 150, 12, 3483, 54.6, 29.4),
        make2L("2xL 150x150x14", 150, 14, 4031, 63.2, 29.3),
        make2L("2xL 150x150x15", 150, 15, 4302, 67.6, 29.3),
        make2L("2xL 150x150x18", 150, 18, 5103, 80.2, 29.2),
        make2L("2xL 160x160x15", 160, 15, 4650, 55.2, 32.4),
        make2L("2xL 180x180x16", 180, 16, 5590, 66.2, 36.4),
        make2L("2xL 200x200x18", 200, 18, 6910, 81.6, 40.5),
    };

    auto byMass = [](const RolledISection &a, const RolledISection &b) {
        return a.mass_kg_per_m < b.mass_kg_per_m;
    };
    std::sort(g_hea.begin(), g_hea.end(), byMass);
    std::sort(g_heb.begin(), g_heb.end(), byMass);
    std::sort(g_ipe.begin(), g_ipe.end(), byMass);

    std::sort(g_2l.begin(), g_2l.end(), [](const DoubleAngleSection &a, const DoubleAngleSection &b) {
        return a.mass_kg_per_m < b.mass_kg_per_m;
    });
}

const std::vector<RolledISection> &heaSections()
{
    initIfNeeded();
    return g_hea;
}

const std::vector<RolledISection> &hebSections()
{
    initIfNeeded();
    return g_heb;
}

const std::vector<RolledISection> &ipeSections()
{
    initIfNeeded();
    return g_ipe;
}

const std::vector<RolledISection> &sectionsForFamily(ColumnFamily f)
{
    switch (f) {
    case ColumnFamily::Heb:
        return hebSections();
    case ColumnFamily::Ipe:
        return ipeSections();
    case ColumnFamily::Hea:
    default:
        return heaSections();
    }
}

const std::vector<DoubleAngleSection> &doubleAngles2L()
{
    initIfNeeded();
    return g_2l;
}

std::optional<DoubleAngleSection> tryGetDoubleAngle2L(QString designation)
{
    initIfNeeded();
    designation = designation.trimmed();
    designation.replace(QStringLiteral("(aşım?)"), QString(), Qt::CaseInsensitive);
    designation.replace(QStringLiteral(" (aşım?)"), QString(), Qt::CaseInsensitive);
    designation = designation.trimmed();
    if (designation.isEmpty() || designation == QStringLiteral("—")) {
        return std::nullopt;
    }
    for (const auto &d : g_2l) {
        if (d.designation.compare(designation, Qt::CaseInsensitive) == 0) {
            return d;
        }
    }
    return std::nullopt;
}

std::optional<RolledISection> tryGetRolledI(QString designation)
{
    initIfNeeded();
    designation = designation.trimmed();
    designation.replace(QStringLiteral("(aşım?)"), QString(), Qt::CaseInsensitive);
    designation.replace(QStringLiteral(" (aşım?)"), QString(), Qt::CaseInsensitive);
    designation = designation.trimmed();
    if (designation.isEmpty() || designation == QStringLiteral("—")) {
        return std::nullopt;
    }
    const auto tryList = [&designation](const std::vector<RolledISection> &v) -> std::optional<RolledISection> {
        for (const auto &s : v) {
            if (s.designation.compare(designation, Qt::CaseInsensitive) == 0) {
                return s;
            }
        }
        return std::nullopt;
    };
    if (auto r = tryList(g_hea)) {
        return r;
    }
    if (auto r = tryList(g_heb)) {
        return r;
    }
    if (auto r = tryList(g_ipe)) {
        return r;
    }
    return std::nullopt;
}

} // namespace SteelCatalog
