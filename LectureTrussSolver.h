#pragma once

#include <QString>

struct PortalFrameInput;
struct PortalFrameResult;
struct SectionOptimizationResult;

struct LectureTrussComparisonStats
{
    int nCompared{0};
    double maxAbsDiffN{0.0};
    double maxRelDiff{0.0};
};

/**
 * Statik belirgin makas / mafsallı düğüm yaklaşımına karşılık gelen 2B pin-eksen doğrusal model.
 * OpenSees’teki beam-column makas ile birebir değildir; çatı hattı düğüm yükleri aynı, kolon rüzgârı
 * bu modelde yoktur (yalnızca çatı q’su düğüm kuvvetine yansır).
 */
class LectureTrussSolver
{
public:
    /** geo.members[].axial_N OpenSees’ten dolu olmalı. Başarıda true; tekil K vb. ise false + mesaj. */
    static bool compareAxialVsOpenSees(const PortalFrameInput &in, const PortalFrameResult &geo,
                                       double q_roof_design_kN_per_m, double w_column_design_kN_per_m,
                                       const SectionOptimizationResult *sizing, QString *outSummary,
                                       LectureTrussComparisonStats *outStats = nullptr);

    /**
     * Makas 2×L kesitlerini iki kaynakla karşılaştırır:
     * - `openSeesAndPinStepSizing`: 1. zarf (OpenSees Tcl + pin-eksen A buradan).
     * - `envelopeAfterPhysical`: fiziksel OpenSees koşuları sonrası STR zarfı (EC3).
     */
    static QString formatTrussSectionsCompareBlock(const SectionOptimizationResult *openSeesAndPinStepSizing,
                                                   const SectionOptimizationResult *envelopeAfterPhysical);
};
