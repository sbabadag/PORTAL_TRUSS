#pragma once

#include "SteelCatalog.h"
#include "TurkishLoads.h"

#include <QString>
#include <vector>

struct PortalFrameInput;
struct PortalFrameResult;

/**
 * Kesit: TS EN 1993-1-1 basitleştirilmiş dayanım (γM0=γM1=1,0) + elastik burkulma χ (eğri b, α=0,34).
 * Kombinasyon zarfı: her STR için ayrı kuvvet alanlarından en büyük kullanım oranı ile profil seçilir.
 */
struct SectionOptimizationResult
{
    QString columnProfile;
    double columnUtilization{0.0}; ///< zarf η (tüm STR)
    /** Makas 2×L: üst/alt hat; köşegen B/C/D sanal bölgeleri (kenar post varsa ayrı). */
    QString trussTopChord2xL;
    double trussTopChordUtilization{0.0};
    QString trussBottomChord2xL;
    double trussBottomChordUtilization{0.0};
    /** Kenar post (makas–çatı kısa düşeyi). */
    QString trussEdgePost2xL;
    double trussEdgePostUtilization{0.0};
    /** Köşegen sanal bölgeleri (sıra w: önce sol sonra sağ; B/C/D katalog taraması başlangıç kesitleri). */
    QString trussWebB2xL;
    double trussWebBUtilization{0.0};
    QString trussWebC2xL;
    double trussWebCUtilization{0.0};
    QString trussWebD2xL;
    double trussWebDUtilization{0.0};
    /** Özet / geriye uyumluluk: en yüksek η; trussProfile2xL kısa özet metin. */
    QString trussProfile2xL;
    double trussUtilization{0.0};
    /** Kolon / makas için zarfı belirleyen kombinasyon kodları (bilgi). */
    QString governingColumnCombinationId;
    QString governingTrussCombinationId;
    QString governingTrussTopCombinationId;
    QString governingTrussBottomCombinationId;
    QString governingTrussEdgePostCombinationId;
    QString governingTrussWebBCombinationId;
    QString governingTrussWebCCombinationId;
    QString governingTrussWebDCombinationId;
    /** Yaklaşık tasarım kuvvetleri (bilgi — kolon zarf sonrası). */
    double approx_N_column_N{0.0};
    double approx_M_column_Nm{0.0};
    double approx_N_truss_max_N{0.0};
    QString envelopeNote;
};

/** Önizleme: tek (qd,wd) ile el formülü veya tek OpenSees sonucu. */
SectionOptimizationResult optimizeSections(const PortalFrameInput &input, double qd_roof_kN_per_m,
                                         double wd_column_kN_per_m, double fy_MPa, ColumnFamily colFamily,
                                         const PortalFrameResult *openSeesResult = nullptr);

/** Tam analiz: her STR kombinasyonu için OpenSees sonuçları; en büyük (ağır) uygun kesit. */
SectionOptimizationResult optimizeSectionsEnvelope(const PortalFrameInput &input,
                                                   const std::vector<PortalFrameResult> &comboResults,
                                                   const std::vector<TurkishLoads::StrCombination> &combos,
                                                   double fy_MPa, ColumnFamily colFamily);
