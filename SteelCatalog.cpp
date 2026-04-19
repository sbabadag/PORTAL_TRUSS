#include "SteelCatalog.h"

#include <algorithm>
#include <cmath>

namespace {

/** A [mm²], Wy [cm³], Iy [cm⁴], Iz [cm⁴], m [kg/m] — EN 10365 tipik değerler. */
RolledISection makeI(const char *name, double A_mm2, double Wy_cm3, double Iy_cm4, double Iz_cm4, double m)
{
    RolledISection s;
    s.designation = QString::fromUtf8(name);
    s.A_m2 = A_mm2 * 1e-6;
    s.Wy_m3 = Wy_cm3 * 1e-6;
    s.Iy_m4 = Iy_cm4 * 1e-8;
    s.Iz_m4 = Iz_cm4 * 1e-8;
    s.mass_kg_per_m = m;
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

    // HEA — EN 10365 tipik Wy [cm³], Iy/Iz [cm⁴] (y-y güçlü, z-z zayıf; kolon burkulmasında Iz kritik)
    g_hea = {
        makeI("HEA 100", 2150, 72.8, 349.0, 26.7, 16.7),      makeI("HEA 120", 2530, 106.3, 606.0, 51.5, 19.9),
        makeI("HEA 140", 3140, 155.4, 1145.0, 116.0, 24.7),   makeI("HEA 160", 3890, 220.1, 2201.0, 228.0, 30.4),
        makeI("HEA 180", 4530, 151.0, 9250.0, 924.0, 35.5),   makeI("HEA 200", 5380, 388.6, 3696.0, 1042.0, 42.3),
        makeI("HEA 220", 6420, 568.5, 5410.0, 1959.0, 50.5), makeI("HEA 240", 7680, 744.6, 7763.0, 2310.0, 60.3),
        makeI("HEA 260", 8680, 836.4, 10450.0, 3660.0, 68.2), makeI("HEA 280", 9730, 949.1, 13170.0, 4763.0, 76.4),
        makeI("HEA 300", 11260, 1118.0, 17170.0, 6310.0, 88.3), makeI("HEA 320", 12400, 1259.0, 20920.0, 7960.0, 97.6),
        makeI("HEA 340", 13360, 1351.0, 24700.0, 10100.0, 105.0),
    };

    // HEB
    g_heb = {
        makeI("HEB 100", 2600, 89.9, 1808.0, 450.0, 20.4),    makeI("HEB 120", 3400, 144.1, 3640.0, 864.0, 26.7),
        makeI("HEB 140", 4330, 216.2, 6050.0, 1510.0, 33.7),  makeI("HEB 160", 5430, 311.0, 9950.0, 2492.0, 42.6),
        makeI("HEB 180", 6530, 426.0, 15000.0, 3830.0, 51.2),  makeI("HEB 200", 7806, 569.0, 18300.0, 5695.0, 61.3),
        makeI("HEB 220", 9100, 735.0, 28400.0, 8090.0, 71.5),  makeI("HEB 240", 10600, 900.0, 36000.0, 11260.0, 83.2),
        makeI("HEB 260", 11810, 1042.0, 47200.0, 14921.0, 93.0), makeI("HEB 280", 13140, 1193.0, 61400.0, 19280.0, 103.0),
        makeI("HEB 300", 14910, 1420.0, 80600.0, 24002.0, 117.0), makeI("HEB 320", 16130, 1540.0, 98500.0, 29520.0, 127.0),
    };

    // IPE — Iy (güçlü), Iz (zayıf) cm⁴
    g_ipe = {
        makeI("IPE 80", 764, 20.0, 801.0, 8.0, 6.0),       makeI("IPE 100", 1030, 34.2, 1710.0, 15.9, 8.1),
        makeI("IPE 120", 1320, 54.7, 3180.0, 27.7, 10.4),   makeI("IPE 140", 1640, 77.3, 5410.0, 45.1, 12.9),
        makeI("IPE 160", 2010, 108.0, 8690.0, 68.3, 15.8),  makeI("IPE 180", 2390, 146.0, 13170.0, 101.0, 18.8),
        makeI("IPE 200", 2848, 194.0, 19430.0, 142.0, 22.4), makeI("IPE 220", 3340, 252.0, 28440.0, 205.0, 26.2),
        makeI("IPE 240", 3912, 324.0, 38920.0, 283.0, 30.7), makeI("IPE 270", 4590, 429.0, 57900.0, 420.0, 36.1),
        makeI("IPE 300", 5380, 557.0, 83560.0, 604.0, 42.2),  makeI("IPE 330", 7130, 713.0, 117700.0, 988.0, 49.1),
        makeI("IPE 360", 7270, 778.0, 162700.0, 1040.0, 57.1), makeI("IPE 400", 8440, 948.0, 231300.0, 1318.0, 66.3),
    };

    // iv [mm] — EN 10056 eşit bacak, asgari atalet yarıçapı (zayıf eksen).
    g_2l = {
        make2L("2xL 45x45x4", 45, 4, 349, 4.2, 10.9),
        make2L("2xL 50x50x5", 50, 5, 480, 5.8, 12.2),
        make2L("2xL 60x60x6", 60, 6, 691, 8.4, 14.6),
        make2L("2xL 70x70x7", 70, 7, 940, 11.4, 16.9),
        make2L("2xL 80x80x8", 80, 8, 1220, 14.8, 15.5),
        make2L("2xL 90x90x9", 90, 9, 1550, 18.8, 17.4),
        make2L("2xL 100x100x10", 100, 10, 1920, 23.2, 19.4),
        make2L("2xL 110x110x10", 110, 10, 2120, 25.6, 21.3),
        make2L("2xL 120x120x11", 120, 11, 2540, 30.6, 23.2),
        make2L("2xL 130x130x12", 130, 12, 3000, 36.0, 25.2),
        make2L("2xL 150x150x14", 150, 14, 4030, 48.4, 30.0),
        make2L("2xL 150x150x15", 150, 15, 4350, 51.6, 30.2),
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

} // namespace SteelCatalog
