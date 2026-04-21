#include "Buckling.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Ec3Buckling {

namespace {

constexpr double kPi = 3.14159265358979323846;

} // namespace

double imperfectionAlpha(ImperfectionCurve curve)
{
    switch (curve) {
    case ImperfectionCurve::A0:
        return 0.13;
    case ImperfectionCurve::A:
        return 0.21;
    case ImperfectionCurve::B:
        return 0.34;
    case ImperfectionCurve::C:
        return 0.49;
    case ImperfectionCurve::D:
        return 0.76;
    }
    return 0.34;
}

double epsilonFactor(double fy_MPa)
{
    const double fy = std::max(fy_MPa, 1e-6);
    return std::sqrt(235.0 / fy);
}

double eulerAxialCriticalForce_N(double E_Pa, double I_m4, double K, double L_m)
{
    const double kl = K * L_m;
    return kPi * kPi * E_Pa * I_m4 / std::max(kl * kl, 1e-18);
}

double lambdaBarFlexuralCompression(double A_m2, double fy_Pa, double Ncr_N)
{
    if (A_m2 < 1e-18 || fy_Pa < 1e-12) {
        return 0.0;
    }
    return std::sqrt(A_m2 * fy_Pa / std::max(Ncr_N, 1e-12));
}

double chiFlexuralBuckling(double lambdaBar, ImperfectionCurve curve)
{
    if (lambdaBar <= 1e-9) {
        return 1.0;
    }
    const double alpha = imperfectionAlpha(curve);
    const double phi = 0.5 * (1.0 + alpha * (lambdaBar - 0.2) + lambdaBar * lambdaBar);
    const double r = phi * phi - lambdaBar * lambdaBar;
    const double den = phi + std::sqrt(std::max(r, 1e-18));
    return std::min(1.0 / den, 1.0);
}

double plasticAxialResistance_N(double A_m2, double fy_Pa, double gammaM0)
{
    return A_m2 * fy_Pa / std::max(gammaM0, 1e-12);
}

double axialBucklingResistance_N(double chi, double A_m2, double fy_Pa, double gammaM1)
{
    return chi * A_m2 * fy_Pa / std::max(gammaM1, 1e-12);
}

double lambdaBarLateralTorsional(double W_m3, double fy_Pa, double Mcr_Nm)
{
    if (W_m3 < 1e-18 || fy_Pa < 1e-12 || Mcr_Nm < 1e-12) {
        return 0.0;
    }
    return std::sqrt(W_m3 * fy_Pa / Mcr_Nm);
}

double chiLateralTorsionalBuckling(double lambdaLT_bar, ImperfectionCurve curveLT)
{
    /** 6.3.2.2: aynı χ(λ̄) biçimi, α_LT olarak Tablo 6.3 değerleri curve ile eşlenir. */
    return chiFlexuralBuckling(lambdaLT_bar, curveLT);
}

double eulerTorsionalFlexuralCriticalForce_N(double E_Pa, double G_Pa, double Iy_m4, double Iz_m4, double It_m4,
                                             double Iw_m6, double K, double L_m, double A_m2)
{
    if (A_m2 < 1e-18 || Iy_m4 + Iz_m4 < 1e-18) {
        return std::numeric_limits<double>::infinity();
    }
    if (It_m4 <= 1e-18 && Iw_m6 <= 1e-18) {
        return std::numeric_limits<double>::infinity();
    }
    if (G_Pa <= 0.0 || L_m < 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    const double kl = std::max(K * L_m, 1e-12);
    const double term = kPi * kPi * E_Pa * std::max(Iw_m6, 0.0) / (kl * kl) + G_Pa * std::max(It_m4, 0.0);
    if (term <= 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    return (A_m2 / (Iy_m4 + Iz_m4)) * term;
}

CrossSectionClass classifyOutstandFlangeCompressionRolled(double fy_MPa, double c_mm, double tf_mm)
{
    if (c_mm <= 0.0 || tf_mm <= 0.0) {
        return CrossSectionClass::Unknown;
    }
    const double eps = epsilonFactor(fy_MPa);
    const double r = c_mm / (tf_mm * eps);
    if (r <= 9.0 + 1e-9) {
        return CrossSectionClass::Class1;
    }
    if (r <= 10.0 + 1e-9) {
        return CrossSectionClass::Class2;
    }
    if (r <= 14.0 + 1e-9) {
        return CrossSectionClass::Class3;
    }
    return CrossSectionClass::Class4;
}

namespace {

/** Tablo 5.2 iç parça: c/(t·ε) üst sınırları, ψ=+1 ve ψ=−1; ara ψ için lineer. */
double internalCompressionLimitCOverTe(double psi, int targetClass)
{
    /** ψ=+1: 33 / 38 / 42; ψ=−1: 72 / 79 / 97 (EN 1993-1-1:2005 Tablo 5.2 sayfa 1). */
    const double p = std::clamp(psi, -1.0, 1.0);
    const double w1 = 0.5 * (1.0 + p);
    const double w2 = 0.5 * (1.0 - p);
    double u1 = 33.0;
    double u2 = 72.0;
    if (targetClass == 2) {
        u1 = 38.0;
        u2 = 79.0;
    } else if (targetClass == 3) {
        u1 = 42.0;
        u2 = 97.0;
    }
    return u1 * w1 + u2 * w2;
}

/** EN 1993-1-5 Tablo 4.1 k_σ(ψ) — ψ −1…+1 lineer. */
double internalPlateKsigma(double psi)
{
    const double p = std::clamp(psi, -1.0, 1.0);
    const double k1 = 4.0;
    const double k0 = 7.81;
    const double km1 = 23.9;
    if (p >= 0.0) {
        return k0 + (k1 - k0) * p;
    }
    return k0 + (k0 - km1) * p;
}

double outstandPlateLambdaP(double cOverT, double eps)
{
    return cOverT / (28.4 * std::max(eps, 1e-9) * std::sqrt(0.43));
}

double internalPlateLambdaP(double cOverT, double eps, double kSigma)
{
    return cOverT / (28.4 * std::max(eps, 1e-9) * std::sqrt(std::max(kSigma, 1e-6)));
}

double rhoOutstand(double lambdaP)
{
    if (lambdaP <= 0.748 + 1e-12) {
        return 1.0;
    }
    return std::clamp((lambdaP - 0.188) / (lambdaP * lambdaP), 0.0, 1.0);
}

double rhoInternal(double lambdaP, double psi)
{
    const double p = std::clamp(psi, -1.0, 1.0);
    if (lambdaP <= 0.673 + 1e-12) {
        return 1.0;
    }
    return std::clamp((lambdaP - 0.055 * (3.0 + p)) / (lambdaP * lambdaP), 0.0, 1.0);
}

} // namespace

CrossSectionClass classifyInternalWebCompression(double fy_MPa, double c_mm, double tw_mm, double psi)
{
    if (c_mm <= 0.0 || tw_mm <= 0.0 || fy_MPa <= 0.0) {
        return CrossSectionClass::Unknown;
    }
    const double eps = epsilonFactor(fy_MPa);
    const double ratio = c_mm / (tw_mm * std::max(eps, 1e-9));
    const double p = std::clamp(psi, -1.0, 1.0);
    const double lim1 = internalCompressionLimitCOverTe(p, 1);
    const double lim2 = internalCompressionLimitCOverTe(p, 2);
    const double lim3 = internalCompressionLimitCOverTe(p, 3);
    if (ratio <= lim1 + 1e-9) {
        return CrossSectionClass::Class1;
    }
    if (ratio <= lim2 + 1e-9) {
        return CrossSectionClass::Class2;
    }
    if (ratio <= lim3 + 1e-9) {
        return CrossSectionClass::Class3;
    }
    return CrossSectionClass::Class4;
}

ImperfectionCurve imperfectionCurveFromOrdinal(int ordinal)
{
    switch (ordinal) {
    case 0:
        return ImperfectionCurve::A0;
    case 1:
        return ImperfectionCurve::A;
    case 2:
        return ImperfectionCurve::B;
    case 3:
        return ImperfectionCurve::C;
    case 4:
        return ImperfectionCurve::D;
    default:
        return ImperfectionCurve::B;
    }
}

double effectiveAreaDoublySymmetricI_m2(double A_gross_m2, double fy_MPa, CrossSectionClass clsFlange,
                                         CrossSectionClass clsWeb, double cf_mm, double tf_mm, double cw_mm,
                                         double tw_mm, double psi_web_for_plate)
{
    if (A_gross_m2 < 1e-18 || fy_MPa <= 0.0 || cf_mm <= 0.0 || tf_mm <= 0.0 || cw_mm <= 0.0 || tw_mm <= 0.0) {
        return A_gross_m2;
    }
    const bool flange4 = (clsFlange == CrossSectionClass::Class4);
    const bool web4 = (clsWeb == CrossSectionClass::Class4);
    if (!flange4 && !web4) {
        return A_gross_m2;
    }
    const double eps = epsilonFactor(fy_MPa);
    const double cOverTf = cf_mm / tf_mm;
    const double cOverTw = cw_mm / tw_mm;
    const double lambdaPf = outstandPlateLambdaP(cOverTf, eps);
    const double kSigW = internalPlateKsigma(psi_web_for_plate);
    const double lambdaPw = internalPlateLambdaP(cOverTw, eps, kSigW);
    const double rhoF = flange4 ? rhoOutstand(lambdaPf) : 1.0;
    const double rhoW = web4 ? rhoInternal(lambdaPw, psi_web_for_plate) : 1.0;
    const double AfGross_mm2 = cf_mm * tf_mm;
    const double AwGross_mm2 = cw_mm * tw_mm;
    const double deltaA_mm2 = 2.0 * (1.0 - rhoF) * AfGross_mm2 + (1.0 - rhoW) * AwGross_mm2;
    const double Aeff = A_gross_m2 - deltaA_mm2 * 1e-6;
    return std::max(Aeff, 1e-9);
}

double effectiveAreaForClass4_m2(double A_gross_m2, CrossSectionClass cls)
{
    if (cls != CrossSectionClass::Class4) {
        return A_gross_m2;
    }
    (void)cls;
    return A_gross_m2;
}

} // namespace Ec3Buckling
