#include "TurkishLoads.h"

namespace TurkishLoads {

double roofLineLoadDesignSnowDominant_kN_per_m(const CharacteristicLineLoadsPerM &c, const Ts498PartialFactors &f)
{
    const double gk = c.dl1 + c.dl2;
    return f.gammaG * gk + f.gammaSnow * c.sn;
}

double roofLineLoadDesignWindLeading_kN_per_m(const CharacteristicLineLoadsPerM &c, const Ts498PartialFactors &f)
{
    const double gk = c.dl1 + c.dl2;
    return f.gammaG * gk + f.gammaSnow * f.psi0SnowWithWind * c.sn;
}

double columnWindLineLoadDesign_kN_per_m(const CharacteristicLineLoadsPerM &c, const Ts498PartialFactors &f)
{
    return f.gammaWind * c.wl;
}

std::vector<StrCombination> strCombinationsDepremDisi(const CharacteristicLineLoadsPerM &c,
                                                      const Ts498PartialFactors &f)
{
    const double gk = c.dl1 + c.dl2;
    std::vector<StrCombination> out;

    StrCombination s1;
    s1.id = QStringLiteral("STR-1");
    s1.description = QStringLiteral("Kar baskın: 1,35Gk + 1,50 Kar (kol rüzgârı yok)");
    s1.q_roof_design_kN_m = f.gammaG * gk + f.gammaSnow * c.sn;
    s1.w_column_design_kN_m = 0.0;
    out.push_back(s1);

    StrCombination s2;
    s2.id = QStringLiteral("STR-2");
    s2.description = QStringLiteral("Rüzgâr baskın (eşlik kar): 1,35Gk + 1,50·ψ0·Kar + 1,50 Rüzgar");
    s2.q_roof_design_kN_m = f.gammaG * gk + f.gammaSnow * f.psi0SnowWithWind * c.sn;
    s2.w_column_design_kN_m = f.gammaWind * c.wl;
    out.push_back(s2);

    StrCombination s3;
    s3.id = QStringLiteral("STR-3");
    s3.description = QStringLiteral("Ölü + rüzgâr (kar yok): 1,35Gk + 1,50 Rüzgar");
    s3.q_roof_design_kN_m = f.gammaG * gk;
    s3.w_column_design_kN_m = f.gammaWind * c.wl;
    out.push_back(s3);

    return out;
}

QString formatRegulatorySummary(const CharacteristicLineLoadsPerM &c, const Ts498PartialFactors &f)
{
    const double gk = c.dl1 + c.dl2;
    const double qSnow = roofLineLoadDesignSnowDominant_kN_per_m(c, f);
    const double qWindLead = roofLineLoadDesignWindLeading_kN_per_m(c, f);
    const double wd = columnWindLineLoadDesign_kN_per_m(c, f);

    return QStringLiteral(
               "TS 498 — yayılı: çatı gk=%1 kN/m (makas otom.+aşık/kapl.+DL2), kar hat=%2 kN/m (SK·0,8·Y); kol wl=%3 kN/m. "
               "STR örnek: qd,çatı≈max(%4,%5) kN/m; wd,kol=%6 kN/m. Katsayıları şartname ile doğrulayın.")
        .arg(gk, 0, 'g', 6)
        .arg(c.sn, 0, 'g', 6)
        .arg(c.wl, 0, 'g', 6)
        .arg(qSnow, 0, 'g', 6)
        .arg(qWindLead, 0, 'g', 6)
        .arg(wd, 0, 'g', 6);
}

} // namespace TurkishLoads
