#pragma once

#include <QString>

struct PortalFrameInput;
struct PortalFrameResult;
struct SectionOptimizationResult;

/**
 * OpenSees Tcl modeli üretir, OPENSEES_EXE veya PATH'teki OpenSees ile çalıştırır,
 * yer değiştirmeleri ve (mümkünse) eleman kuvvetlerini okur.
 *
 * Konsoldan (cmd/PowerShell) çalıştırırken stderr’e aşamalar ve OpenSees metin çıktısı akar.
 * Çift tıklamada kapatmak için: PORTAL_OPENSEES_CONSOLE=0. Zorla açmak için: =1.
 */
/**
 * Tasarım yükleri (kN/m): çatı/makas hattı ve kolon rüzgârı — TS 498 STR kombinasyonuna göre ayrı ayrı verilir.
 * trussPhysicalSizing: birinci zarf sonrası kesit özeti; verilirse makas elemanlarında A (ve 2×L için I) katalogdan.
 * Hadde I (IPE/HEA/HEB) seçiliyken makaslar OpenSees `truss` (tek eksenel); gusset şeridi `elasticBeamColumn`.
 */
bool runOpenSeesStaticAnalysis(const PortalFrameInput &input, PortalFrameResult &ioResult,
                               double q_roof_design_kN_m, double w_column_design_kN_m, QString *errorOut,
                               const SectionOptimizationResult *trussPhysicalSizing = nullptr);
