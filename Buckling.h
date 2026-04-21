#pragma once

#include <cmath>
#include <cstdint>

/**
 * EN 1993-1-1 burkulma altyapısı (üye düzeyi + ileride kesit/yerel).
 *
 * Uygulanan:
 *   - 6.3.1.2 — eksenel eğilme burkulması: λ̄, χ (eğri A0…D).
 *   - Euler eksenel kritik kuvvet Ncr = π²EI / (KL)².
 *   - Basınç: Nb,Rd = χ A fy / γM1; çekme: Npl,Rd = A fy / γM0.
 *   - 6.3.1.3 — Ncr,T (It, Iw katalogda; G = E/(2(1+ν))).
 *   - 5.5 — hadde dış flans + iç gövde sınıfı (Tablo 5.2); sınıf 4 için EN 1993-1-5 etkin alan (basitleştirilmiş).
 *
 * İskelet / sınırlı:
 *   - 6.3.2 — LTB χ_LT (Mcr girilince); portal kolon z-z eğilmesinde LTB tipik olarak devreye alınmaz.
 */
namespace Ec3Buckling {

/** Tablo 6.1 / 6.2 imperfection eğrileri — α değerleri. */
enum class ImperfectionCurve : uint8_t
{
    A0 = 0, ///< α = 0,13
    A,      ///< 0,21
    B,      ///< 0,34 — hadde profil eksenel eğilme için tipik
    C,      ///< 0,49
    D       ///< 0,76
};

/** Eğri için α (EN 1993-1-1 Tablo 6.1). */
double imperfectionAlpha(ImperfectionCurve curve);

/** ε = √(235 / fy[MPa]) — flans/web sınıf limitleri (5.5) için. */
double epsilonFactor(double fy_MPa);

/** Euler: Ncr = π² E I / (K L)² [N]. */
double eulerAxialCriticalForce_N(double E_Pa, double I_m4, double K, double L_m);

/** λ̄ = √(A fy / Ncr) — basınç üyesi eksenel eğilme (6.3.1). */
double lambdaBarFlexuralCompression(double A_m2, double fy_Pa, double Ncr_N);

/** 6.3.1.2 — χ düşürme faktörü. */
double chiFlexuralBuckling(double lambdaBar, ImperfectionCurve curve);

/** Npl,Rd = A fy / γM0 [N]. */
double plasticAxialResistance_N(double A_m2, double fy_Pa, double gammaM0);

/** Nb,Rd = χ A fy / γM1 [N]. */
double axialBucklingResistance_N(double chi, double A_m2, double fy_Pa, double gammaM1);

// --- 6.3.2 Burulmalı burkulma (LTB) — χ_LT aynı φ yapısı, farklı α_LT ---

/** λ̄_LT = √(W fy / Mcr); Mcr elastik kritik moment [N·m]. W: elastik veya plastik modül [m³]. */
double lambdaBarLateralTorsional(double W_m3, double fy_Pa, double Mcr_Nm);

/**
 * 6.3.2.2 χ_LT — φ formülü 6.3.1.2 ile aynı, α_LT Tablo 6.3 (hadde: genelde 0,34).
 * curveLT: pratikte B veya C seçilir; çift açı için C daha konservatif olabilir.
 */
double chiLateralTorsionalBuckling(double lambdaLT_bar, ImperfectionCurve curveLT);

// --- 6.3.1.3 Burulmalı / burulmalı-eğilme eksenel (çift simetrik I yaklaşımı) ---

/**
 * Çift simetrik I için tipik ifade: Ncr,T ≈ (A/(Iy+Iz)) · (π²EIw/(KL)² + G It) [N].
 * It, Iw veya G ≤ 0 ise kontrol dışı bırakılır (+∞ döner, min(Ncr) ile elenir).
 */
double eulerTorsionalFlexuralCriticalForce_N(double E_Pa, double G_Pa, double Iy_m4, double Iz_m4, double It_m4,
                                             double Iw_m6, double K, double L_m, double A_m2);

// --- 5.5 Kesit sınıfı (yerel burkulma öncesi) ---

enum class CrossSectionClass : int
{
    Unknown = 0,
    Class1 = 1,
    Class2 = 2,
    Class3 = 3,
    Class4 = 4
};

/**
 * Hadde profil — dışta basınç taşıyan flans (Table 5.2, rolled). c = çıkıntı genişliği [mm], tf [mm].
 * Geçersiz boyut → Unknown.
 */
CrossSectionClass classifyOutstandFlangeCompressionRolled(double fy_MPa, double c_mm, double tf_mm);

/**
 * İç basınç gövdesi — EN 1993-1-1 Tablo 5.2 (sayfa 1), ψ = σ₂/σ₁ kenar gerilmeleri (−1…+1, basınç +).
 * ψ = +1 düzgün basınç; ψ = −1 saf eğilme (doğrusal); ara değerler için sınır oranları lineer enterpolasyon.
 */
CrossSectionClass classifyInternalWebCompression(double fy_MPa, double c_mm, double tw_mm, double psi);

/** Tablo 6.1 eğrisi: ordinal 0…4 = A0…D; aralık dışı → B. */
ImperfectionCurve imperfectionCurveFromOrdinal(int ordinal);

/**
 * Hadde I — çift simetrik, düzgün basınç gövdesi (ψ=+1) ve dışta basınç flansları.
 * Sınıf 4 parça için EN 1993-1-5 etkin genişlik (4.1–4.3); flans/gövde ≤3 ise ρ=1.
 * Geçersiz boyut veya fy → brüt A.
 */
double effectiveAreaDoublySymmetricI_m2(double A_gross_m2, double fy_MPa, CrossSectionClass clsFlange,
                                         CrossSectionClass clsWeb, double cf_mm, double tf_mm, double cw_mm,
                                         double tw_mm, double psi_web_for_plate);

/** Eski imza: yalnızca kesit sınıfı — geometri yoksa brüt A. */
double effectiveAreaForClass4_m2(double A_gross_m2, CrossSectionClass cls);

} // namespace Ec3Buckling
