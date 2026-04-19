#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "SectionOptimizer.h"
#include "SteelCatalog.h"
#include "TurkishLoads.h"

#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cmath>

#include <QBrush>
#include <QFont>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QPainter>
#include <QPen>
#include <QMessageBox>
#include <QPolygonF>
#include <QResizeEvent>
#include <QGraphicsScene>
#include <QComboBox>

namespace {

QPointF scenePoint(double x, double y, double scale, double originX, double originY)
{
    return QPointF(originX + x * scale, originY - y * scale);
}

void addArrowHeadAtTip(QGraphicsScene *scene, const QPointF &tip, const QPointF &shaftDir, const QColor &col)
{
    double L = std::hypot(shaftDir.x(), shaftDir.y());
    if (L < 1e-9) {
        return;
    }
    QPointF u = shaftDir / L;
    QPointF perp(-u.y(), u.x());
    const double h = 6.0;
    const double w = 3.5;
    const QPointF p1 = tip - u * h + perp * w;
    const QPointF p2 = tip - u * h - perp * w;
    scene->addPolygon(QPolygonF({tip, p1, p2}), QPen(col), QBrush(col));
}

void drawVerticalDistributedOnSegment(QGraphicsScene *scene, double x0, double y0, double x1, double y1, int nStations,
                                      double scale, double ox, double oy, const QColor &col, double structArrowLen)
{
    const QPen pen(col, 1.4);
    for (int i = 0; i < nStations; ++i) {
        const double t = (static_cast<double>(i) + 0.5) / static_cast<double>(nStations);
        const double x = x0 + t * (x1 - x0);
        const double y = y0 + t * (y1 - y0);
        const QPointF pBeam = scenePoint(x, y, scale, ox, oy);
        const QPointF pTip = scenePoint(x, y - structArrowLen, scale, ox, oy);
        scene->addLine(QLineF(pBeam, pTip), pen);
        addArrowHeadAtTip(scene, pTip, pTip - pBeam, col);
    }
}

/** Horizontal uniform line load on a column: arrows toward the column from outside. */
void drawHorizontalDistributedOnColumn(QGraphicsScene *scene, double xCol, double y0, double y1, int nStations,
                                       double scale, double ox, double oy, const QColor &col, double structLen,
                                       bool fromLeft)
{
    const QPen pen(col, 1.4);
    for (int i = 0; i < nStations; ++i) {
        const double t = (static_cast<double>(i) + 0.5) / static_cast<double>(nStations);
        const double y = y0 + t * (y1 - y0);
        const QPointF pEnd = scenePoint(xCol, y, scale, ox, oy);
        const QPointF pStart = fromLeft ? scenePoint(xCol - structLen, y, scale, ox, oy)
                                        : scenePoint(xCol + structLen, y, scale, ox, oy);
        scene->addLine(QLineF(pStart, pEnd), pen);
        addArrowHeadAtTip(scene, pEnd, pEnd - pStart, col);
    }
}

QString stripProfileSuffix(const QString &s)
{
    QString t = s;
    t.replace(QStringLiteral(" (aşım?)"), QString());
    t.replace(QStringLiteral("(aşım?)"), QString());
    return t.trimmed();
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->combo_columnFamily->addItems({QStringLiteral("HEA"), QStringLiteral("HEB"), QStringLiteral("IPE")});
    ui->combo_columnFamily->setCurrentIndex(1); // varsayılan test: HEB

    m_scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(m_scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing, true);
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);

    connect(ui->push_calculate, &QPushButton::clicked, this, &MainWindow::onCalculateClicked);
    connect(&m_watcher, &QFutureWatcher<PortalFrameResult>::finished, this, &MainWindow::onAnalysisFinished);

    setupScene();

    connect(ui->spin_span, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_columnHeight, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_panels, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) { setupScene(); });
    connect(ui->spin_trussAxisY, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_area, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_purlinClad, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_dl2, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_sk, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->spin_wl, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });
    connect(ui->combo_columnFamily, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { setupScene(); });
    connect(ui->spin_fy, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) { setupScene(); });

    ui->spin_apexHeight->setReadOnly(true);
    ui->spin_apexHeight->setToolTip(QStringLiteral(
        "Kağıttaki L = tam açıklık (Span width). Kolon yüzünde L/40, ortada makas derinliği L/11; tepe otomatik."));
}

MainWindow::~MainWindow()
{
    m_watcher.waitForFinished();
}

void MainWindow::setupScene()
{
    PortalFrameInput in = collectInput();
    ui->spin_apexHeight->blockSignals(true);
    ui->spin_apexHeight->setValue(in.apexHeight_m);
    ui->spin_apexHeight->blockSignals(false);

    PortalFrameResult preview;
    PortalSolver::buildPortalGeometry(in, preview);
    drawFrameFromResult(preview, in);
}

PortalFrameInput MainWindow::collectInput() const
{
    PortalFrameInput in;
    in.spanWidth_m = ui->spin_span->value();
    in.columnHeight_m = ui->spin_columnHeight->value();
    in.apexHeight_m = PortalSolver::trussApexHeight_m(in);
    in.trussPanelsPerSide = ui->spin_panels->value();
    in.trussAxisSpacingY_m = ui->spin_trussAxisY->value();
    in.steelArea_m2 = ui->spin_area->value();
    in.steelInertia_m4 = ui->spin_inertia->value();
    in.youngModulus_Pa = ui->spin_E->value();
    in.purlinCladding_kN_per_m2 = ui->spin_purlinClad->value();
    in.dl2_kN_per_m2 = ui->spin_dl2->value();
    in.sk_kN_per_m2 = ui->spin_sk->value();
    in.wl_kN_per_m2 = ui->spin_wl->value();
    in.columnFamilyIndex = ui->combo_columnFamily->currentIndex();
    in.fy_MPa = ui->spin_fy->value();
    return in;
}

void MainWindow::drawFrameFromResult(const PortalFrameResult &result, const PortalFrameInput &input)
{
    ui->label_trussDeadAuto->setText(QString::number(PortalSolver::trussSelfWeightHoriz_kN_per_m2(input), 'f', 3));
    m_scene->clear();

    if (result.nodes.empty()) {
        return;
    }

    double minX = result.nodes.front().x;
    double maxX = result.nodes.front().x;
    double minY = result.nodes.front().y;
    double maxY = result.nodes.front().y;
    for (const auto &n : result.nodes) {
        minX = std::min(minX, n.x);
        maxX = std::max(maxX, n.x);
        minY = std::min(minY, n.y);
        maxY = std::max(maxY, n.y);
    }

    const double pad = 1.0;
    const double spanData = std::max(1e-6, maxX - minX + 2.0 * pad);
    const double heightData = std::max(1e-6, maxY - minY + 2.0 * pad);

    const QRect vr = ui->graphicsView->viewport()->rect();
    const double sx = (vr.width() - 40.0) / spanData;
    const double sy = (vr.height() - 40.0) / heightData;
    const double scale = std::max(1e-6, std::min(sx, sy));

    const double originX = 20.0 + (-minX + pad) * scale;
    const double originY = vr.height() - 20.0 - (-minY + pad) * scale;

    // Çizim renkleri — bilgi kutusundaki efsane ile aynı kodlar
    constexpr double kPenChord = 2.85;
    constexpr double kPenBrace = 1.12;
    const QColor colBottomChord(21, 101, 192);  // mavi — alt hat
    const QColor colTopChord(27, 94, 32);       // yeşil — üst hat
    const QColor colWeb(216, 27, 96);             // genel köşegen
    const QColor colWebB(142, 36, 62);            // sanal B (60×60×6 başlangıç)
    const QColor colWebC(106, 27, 154);         // sanal C (50×50×5)
    const QColor colWebD(180, 90, 30);            // sondan önceki (60×60×6)
    QPen penChordBottom(colBottomChord, kPenChord);
    constexpr double kPenGusset = 2.35;
    const QColor colGussetStrip(0, 105, 125);
    QPen penGussetStrip(colGussetStrip, kPenGusset);
    QPen penChordTop(colTopChord, kPenChord);
    QPen penWeb(colWeb, kPenBrace);
    QPen penWebB(colWebB, kPenBrace);
    QPen penWebC(colWebC, kPenBrace);
    QPen penWebD(colWebD, kPenBrace);
    QPen columnPen(QColor(40, 40, 40), 2.65);

    auto nodeXY = [&](int tag, double &ox, double &oy) {
        for (const auto &n : result.nodes) {
            if (n.tag == tag) {
                ox = n.x;
                oy = n.y;
                return true;
            }
        }
        return false;
    };

    const TurkishLoads::Ts498PartialFactors part;
    const auto ch = input.lineLoadsCharacteristic();
    const auto strCombos = TurkishLoads::strCombinationsDepremDisi(ch, part);
    double qdRoof = 0.0;
    double wdCol = 0.0;
    for (const auto &co : strCombos) {
        qdRoof = std::max(qdRoof, co.q_roof_design_kN_m);
        wdCol = std::max(wdCol, co.w_column_design_kN_m);
    }
    const SectionOptimizationResult opt =
        result.ok ? result.sectionDesign
                  : optimizeSections(input, qdRoof, wdCol, input.fy_MPa,
                                     static_cast<ColumnFamily>(input.columnFamilyIndex));

    for (const auto &m : result.members) {
        double x1 = 0.0;
        double y1 = 0.0;
        double x2 = 0.0;
        double y2 = 0.0;
        if (!nodeXY(m.nodeI, x1, y1) || !nodeXY(m.nodeJ, x2, y2)) {
            continue;
        }
        const QPointF p1 = scenePoint(x1, y1, scale, originX, originY);
        const QPointF p2 = scenePoint(x2, y2, scale, originX, originY);
        const QLineF line(p1, p2);

        QPen penUse = columnPen;
        if (m.isTruss) {
            switch (m.trussRole) {
            case TrussMemberRole::ChordBottom:
                penUse = penChordBottom;
                break;
            case TrussMemberRole::GussetStrip:
                penUse = penGussetStrip;
                break;
            case TrussMemberRole::ChordTop:
                penUse = penChordTop;
                break;
            case TrussMemberRole::EdgePost:
                penUse = penWeb;
                break;
            case TrussMemberRole::Web:
                switch (m.webOptZone) {
                case 0:
                    penUse = penChordTop;
                    break;
                case 1:
                    penUse = penWebB;
                    break;
                case 2:
                    penUse = penWebC;
                    break;
                case 3:
                    penUse = penWebD;
                    break;
                default:
                    penUse = penWeb;
                    break;
                }
                break;
            default:
                penUse = penWeb;
                break;
            }
        }
        auto *li = m_scene->addLine(line, penUse);
        li->setZValue(1);
    }

    const double nodeR = std::max(2.0, scale * 0.08);
    for (const auto &n : result.nodes) {
        const QPointF c = scenePoint(n.x, n.y, scale, originX, originY);
        auto *dot = m_scene->addEllipse(c.x() - nodeR, c.y() - nodeR, 2 * nodeR, 2 * nodeR, QPen(Qt::darkGray, 1.0),
                                          QBrush(QColor(255, 200, 60)));
        dot->setZValue(2);
    }

    const double W = input.spanWidth_m;
    const double Hc = input.columnHeight_m;
    const double halfW = 0.5 * W;
    const double Ha = PortalSolver::trussApexHeight_m(input);

    // TS 498: roof — uniform vertical line load (dead + snow) along each main roof slope (global düşey gösterim).
    // Üst hat kolon seviyesinde (Hc) başlar.
    const double structVert = std::max(0.2, std::min(0.45, (Ha - Hc) * 0.12 + 0.15));
    const int nRoof = 9;
    const QColor roofLoadCol(180, 30, 30);
    drawVerticalDistributedOnSegment(m_scene, 0.0, Hc, halfW, Ha, nRoof, scale, originX, originY, roofLoadCol,
                                     structVert);
    drawVerticalDistributedOnSegment(m_scene, halfW, Ha, W, Hc, nRoof, scale, originX, originY, roofLoadCol,
                                     structVert);

    auto *roofLbl = m_scene->addText(QStringLiteral("qd,çatı = %1 kN/m (STR)").arg(qdRoof, 0, 'f', 2));
    roofLbl->setDefaultTextColor(roofLoadCol);
    roofLbl->setFont(QFont(QStringLiteral("Segoe UI"), 9, QFont::Bold));
    roofLbl->setPos(scenePoint(halfW, Ha, scale, originX, originY).x() - 40.0,
                    scenePoint(halfW, Ha, scale, originX, originY).y() - 28.0);
    roofLbl->setZValue(4);

    // TS 498: wind — uniform horizontal line load along each column height.
    const double structHorz = std::max(0.25, std::min(0.5, Hc * 0.08));
    const int nCol = 7;
    const QColor windCol(0, 90, 150);
    drawHorizontalDistributedOnColumn(m_scene, 0.0, 0.0, Hc, nCol, scale, originX, originY, windCol, structHorz,
                                      true);
    drawHorizontalDistributedOnColumn(m_scene, W, 0.0, Hc, nCol, scale, originX, originY, windCol, structHorz, false);

    auto *windLbl = m_scene->addText(QStringLiteral("wd,kol = %1 kN/m (STR)").arg(wdCol, 0, 'f', 2));
    windLbl->setDefaultTextColor(windCol);
    windLbl->setFont(QFont(QStringLiteral("Segoe UI"), 9, QFont::Bold));
    windLbl->setPos(scenePoint(0.0, 0.55 * Hc, scale, originX, originY).x() - 10.0,
                    scenePoint(0.0, 0.55 * Hc, scale, originX, originY).y() - 8.0);
    windLbl->setZValue(4);

    const QString cCol = stripProfileSuffix(opt.columnProfile);
    const QString cBot = stripProfileSuffix(opt.trussBottomChord2xL);
    const QString cTop = stripProfileSuffix(opt.trussTopChord2xL);
    const QString cWB = stripProfileSuffix(opt.trussWebB2xL);
    const QString cWC = stripProfileSuffix(opt.trussWebC2xL);
    const QString cWD = stripProfileSuffix(opt.trussWebD2xL);

    const QString html =
        QStringLiteral("<div style='text-align:left'>"
                       "<div style='font-weight:700;font-size:11pt;margin-bottom:6px'>Kesit (TS 498 STR zarf, TS EN "
                       "1993-1-1)</div>"
                       "<div style='font-size:9pt;line-height:1.45;color:#222'>"
                       "<b>Renk — eleman</b><br/>"
                       "<span style='color:#282828'>■</span> Kolon<br/>"
                       "<span style='color:#1565c0'>■</span> Alt hat<br/>"
                       "<span style='color:#00697d'>■</span> Uç gusset (kısa düşey, alt hat 2×L)<br/>"
                       "<span style='color:#1b5e20'>■</span> Üst hat + köşegen w=0,1 (üst hat 2×L)<br/>"
                       "<span style='color:#8e243e'>■</span> Köşegen B (w=2…7, 60×60×6’dan arama)<br/>"
                       "<span style='color:#6a1b9a'>■</span> Köşegen C (w=8…15, 50×50×5’ten)<br/>"
                       "<span style='color:#b45a1e'>■</span> Köşegen D (sondan bir önceki w)<br/>"
                       "<span style='font-size:8pt;color:#555'>w: her j’de önce sol sonra sağ çapraz; toplam 2N çubuk.</span>"
                       "</div>"
                       "<div style='margin-top:8px;font-size:10pt;line-height:1.4'>"
                       "<span style='color:#282828'>■</span> Kolon: %1 — η=%2 <span style='color:#666'>(%3)</span><br/>"
                       "<span style='color:#1565c0'>■</span> Alt hat: %4 — η=%5 <span "
                       "style='color:#666'>(%6)</span><br/>"
                       "<span style='color:#1b5e20'>■</span> Üst hat: %7 — η=%8 <span "
                       "style='color:#666'>(%9)</span><br/>"
                       "<span style='color:#8e243e'>■</span> Köşegen B: %10 — η=%11 <span "
                       "style='color:#666'>(%12)</span><br/>"
                       "<span style='color:#6a1b9a'>■</span> Köşegen C: %13 — η=%14 <span "
                       "style='color:#666'>(%15)</span><br/>"
                       "<span style='color:#b45a1e'>■</span> Köşegen D: %16 — η=%17 <span "
                       "style='color:#666'>(%18)</span>"
                       "</div>"
                       "<div style='margin-top:8px;font-size:9pt;color:#444'>%19</div>"
                       "</div>")
            .arg(cCol)
            .arg(opt.columnUtilization, 0, 'f', 2)
            .arg(opt.governingColumnCombinationId.isEmpty() ? QStringLiteral("—")
                                                             : opt.governingColumnCombinationId)
            .arg(cBot)
            .arg(opt.trussBottomChordUtilization, 0, 'f', 2)
            .arg(opt.governingTrussBottomCombinationId.isEmpty() ? QStringLiteral("—")
                                                                   : opt.governingTrussBottomCombinationId)
            .arg(cTop)
            .arg(opt.trussTopChordUtilization, 0, 'f', 2)
            .arg(opt.governingTrussTopCombinationId.isEmpty() ? QStringLiteral("—") : opt.governingTrussTopCombinationId)
            .arg(cWB)
            .arg(opt.trussWebBUtilization, 0, 'f', 2)
            .arg(opt.governingTrussWebBCombinationId.isEmpty() ? QStringLiteral("—")
                                                                  : opt.governingTrussWebBCombinationId)
            .arg(cWC)
            .arg(opt.trussWebCUtilization, 0, 'f', 2)
            .arg(opt.governingTrussWebCCombinationId.isEmpty() ? QStringLiteral("—")
                                                                   : opt.governingTrussWebCCombinationId)
            .arg(cWD)
            .arg(opt.trussWebDUtilization, 0, 'f', 2)
            .arg(opt.governingTrussWebDCombinationId.isEmpty() ? QStringLiteral("—")
                                                                   : opt.governingTrussWebDCombinationId)
            .arg(opt.envelopeNote.isEmpty() ? QStringLiteral("Önizleme: üst sınır q,w ile el hesap.")
                                             : opt.envelopeNote);

    auto *info = m_scene->addText(QString());
    info->setHtml(html);
    info->setDefaultTextColor(QColor(25, 25, 25));
    info->setTextWidth(380.0);
    const QRectF br = info->boundingRect();
    const QRect vrLegend = ui->graphicsView->viewport()->rect();
    const int margin = 10;
    const QPointF topRightScene = ui->graphicsView->mapToScene(QPoint(vrLegend.width() - margin, margin));
    info->setPos(topRightScene.x() - br.width(), topRightScene.y());
    const QRectF frame = br.translated(info->pos()).adjusted(-10, -8, 10, 8);
    auto *bg = m_scene->addRect(frame, QPen(QColor(160, 160, 170), 1.0), QBrush(QColor(255, 255, 255, 228)));
    bg->setZValue(9);
    info->setZValue(10);

    m_scene->setSceneRect(m_scene->itemsBoundingRect().adjusted(-10, -10, 10, 10));
}

void MainWindow::onCalculateClicked()
{
    if (m_busy) {
        return;
    }

    ui->label_status->setText(tr("Running analysis…"));
    ui->push_calculate->setEnabled(false);
    m_busy = true;

    const PortalFrameInput input = collectInput();
    QFuture<PortalFrameResult> fut = QtConcurrent::run([input]() {
        PortalSolver solver;
        return solver.analyze(input);
    });
    m_watcher.setFuture(fut);
}

void MainWindow::onAnalysisFinished()
{
    m_busy = false;
    ui->push_calculate->setEnabled(true);

    const PortalFrameResult r = m_watcher.future().result();
    if (!r.ok) {
        ui->label_status->setText(tr("Error: %1").arg(r.errorMessage));
        QMessageBox::warning(this, tr("Analysis"), r.errorMessage);
        return;
    }

    ui->label_status->setText(r.errorMessage);
    drawFrameFromResult(r, collectInput());
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    if (!m_busy) {
        setupScene();
    }
}
