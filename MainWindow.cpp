#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "SectionOptimizer.h"
#include "SteelCatalog.h"
#include "TurkishLoads.h"

#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

#include <QBrush>
#include <QFont>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QGraphicsPolygonItem>
#include <QPainter>
#include <QPen>
#include <QMessageBox>
#include <QPolygonF>
#include <QResizeEvent>
#include <QGraphicsScene>
#include <QTextEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QLineEdit>
#include <QLabel>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

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

QString memberProfileLabel(const MemberResult &m, const SectionOptimizationResult &opt)
{
    if (!m.isTruss) {
        return stripProfileSuffix(opt.columnProfile);
    }
    switch (m.trussRole) {
    case TrussMemberRole::ChordBottom:
    case TrussMemberRole::GussetStrip:
        return stripProfileSuffix(opt.trussBottomChord2xL);
    case TrussMemberRole::ChordTop:
        return stripProfileSuffix(opt.trussTopChord2xL);
    case TrussMemberRole::EdgePost:
        return stripProfileSuffix(opt.trussEdgePost2xL);
    case TrussMemberRole::Web:
        switch (m.webOptZone) {
        case 0:
            return stripProfileSuffix(opt.trussTopChord2xL);
        case 1:
            return stripProfileSuffix(opt.trussWebB2xL);
        case 2:
            return stripProfileSuffix(opt.trussWebC2xL);
        case 3:
            return stripProfileSuffix(opt.trussWebD2xL);
        default:
            return stripProfileSuffix(opt.trussWebC2xL);
        }
    default:
        return {};
    }
}

} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Sol panel formu pencere yüksekliğini doldursun; sağda çizim alanı esnesin.
    ui->horizontalLayout_main->setStretch(0, 0);
    ui->horizontalLayout_main->setStretch(1, 1);
    ui->verticalLayout_sidebar->setStretch(0, 0);
    ui->verticalLayout_sidebar->setStretch(1, 1);
    ui->verticalLayout_sidebar->setStretch(2, 0);
    ui->verticalLayout_sidebar->setStretch(3, 0);
    ui->verticalLayout_view->setStretch(0, 0);
    ui->verticalLayout_view->setStretch(1, 1);
    ui->verticalLayout_view->setStretch(2, 0);

    ui->combo_columnFamily->addItems({QStringLiteral("HEA"), QStringLiteral("HEB"), QStringLiteral("IPE")});
    ui->combo_columnFamily->setCurrentIndex(1); // varsayılan test: HEB

    m_comboColumnBase = new QComboBox(ui->scrollAreaWidgetContents);
    m_comboColumnBase->addItem(tr("Ankastre (fixed)"), static_cast<int>(ColumnBaseSupport::Fixed));
    m_comboColumnBase->addItem(tr("Mafsallı (pinned)"), static_cast<int>(ColumnBaseSupport::Pinned));
    m_comboColumnBase->setCurrentIndex(0);
    m_comboColumnBase->setToolTip(
        tr("Statik model: ankastre tabanda ux, uy ve dönme tutulu; mafsallıda tabanda dönme serbest. "
           "OpenSees kolon/gusset ayakları buna göre fixlenir."));
    auto *lblColBase = new QLabel(tr("Kolon tabanı"), ui->scrollAreaWidgetContents);
    lblColBase->setToolTip(m_comboColumnBase->toolTip());
    ui->formLayout->insertRow(5, lblColBase, m_comboColumnBase);

    m_lineColumnZzBuckling = new QLineEdit(ui->scrollAreaWidgetContents);
    m_lineColumnZzBuckling->setPlaceholderText(tr("e.g. 0.33, 0.66"));
    m_lineColumnZzBuckling->setToolTip(
        tr("Z-z eksenel burkulma (Euler Iz) için tutuluş h/H (0..1), virgülle.\n"
           "Doluysa yalnızca bu oranlar kullanılır (elle/makas listesi z-z için devre dışı).\n"
           "Boşsa: «Lateral brace h/H» + isteğe bağlı makas geometrisi birleşimi kullanılır."));
    auto *lblZzBuck = new QLabel(tr("Z-z buckling brace h/H"), ui->scrollAreaWidgetContents);
    lblZzBuck->setToolTip(m_lineColumnZzBuckling->toolTip());
    ui->formLayout->insertRow(24, lblZzBuck, m_lineColumnZzBuckling);

    m_lineColumnLtbBottom = new QLineEdit(ui->scrollAreaWidgetContents);
    m_lineColumnLtbBottom->setPlaceholderText(tr("e.g. 0.5"));
    m_lineColumnLtbBottom->setToolTip(
        tr("LTB: alt başlık (flanş) yanal tutuluş h/H — Mcr ve χ_LT (güçlü eksen eğilme)."));
    auto *lblLtbB = new QLabel(tr("LTB alt başlık h/H"), ui->scrollAreaWidgetContents);
    lblLtbB->setToolTip(m_lineColumnLtbBottom->toolTip());
    ui->formLayout->insertRow(25, lblLtbB, m_lineColumnLtbBottom);

    m_lineColumnLtbTop = new QLineEdit(ui->scrollAreaWidgetContents);
    m_lineColumnLtbTop->setPlaceholderText(tr("e.g. 0.5"));
    m_lineColumnLtbTop->setToolTip(
        tr("LTB: üst başlık yanal tutuluş h/H — Mcr ve χ_LT.\n"
           "Alt ve üst doluysa Mcr ≈ min(Mcr alt, Mcr üst) (muhafazakâr)."));
    auto *lblLtbT = new QLabel(tr("LTB üst başlık h/H"), ui->scrollAreaWidgetContents);
    lblLtbT->setToolTip(m_lineColumnLtbTop->toolTip());
    ui->formLayout->insertRow(26, lblLtbT, m_lineColumnLtbTop);

    m_comboColumnLtbCurve = new QComboBox(ui->scrollAreaWidgetContents);
    m_comboColumnLtbCurve->addItems({QStringLiteral("A0 (α=0,13)"), QStringLiteral("A (0,21)"),
                                     QStringLiteral("B (0,34)"), QStringLiteral("C (0,49)"),
                                     QStringLiteral("D (0,76)")});
    m_comboColumnLtbCurve->setCurrentIndex(2);
    m_comboColumnLtbCurve->setToolTip(tr("LTB χ_LT için imperfection eğrisi (Tablo 6.3 yaklaşımı)."));
    auto *lblLtbCurve = new QLabel(tr("LTB χ curve"), ui->scrollAreaWidgetContents);
    lblLtbCurve->setToolTip(m_comboColumnLtbCurve->toolTip());
    ui->formLayout->insertRow(27, lblLtbCurve, m_comboColumnLtbCurve);

    const QStringList bucklingCurves = {QStringLiteral("A0 (α=0,13)"), QStringLiteral("A (0,21)"),
                                        QStringLiteral("B (0,34)"), QStringLiteral("C (0,49)"),
                                        QStringLiteral("D (0,76)")};
    ui->combo_colBucklingCurve->addItems(bucklingCurves);
    ui->combo_trussBucklingCurve->addItems(bucklingCurves);
    ui->combo_colBucklingCurve->setCurrentIndex(2);
    ui->combo_trussBucklingCurve->setCurrentIndex(2);

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
    connect(ui->spin_colBucklingKy, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { setupScene(); });
    connect(ui->spin_colBucklingKz, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { setupScene(); });
    connect(ui->combo_colBucklingCurve, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { setupScene(); });
    connect(ui->combo_trussBucklingCurve, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { setupScene(); });
    connect(ui->check_colBraceFromTruss, &QCheckBox::toggled, this, [this](bool) { setupScene(); });
    connect(ui->line_colBraceHeights, &QLineEdit::editingFinished, this, [this]() { setupScene(); });
    connect(m_lineColumnZzBuckling, &QLineEdit::editingFinished, this, [this]() { setupScene(); });
    connect(m_lineColumnLtbBottom, &QLineEdit::editingFinished, this, [this]() { setupScene(); });
    connect(m_lineColumnLtbTop, &QLineEdit::editingFinished, this, [this]() { setupScene(); });
    connect(m_comboColumnLtbCurve, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { setupScene(); });
    connect(ui->combo_columnFamily, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { setupScene(); });
    connect(m_comboColumnBase, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) { setupScene(); });
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
    in.columnBaseSupport =
        static_cast<ColumnBaseSupport>(m_comboColumnBase->currentData().toInt());
    in.columnLtbCurveOrdinal = m_comboColumnLtbCurve ? m_comboColumnLtbCurve->currentIndex() : 2;
    auto parseFracList = [](const QString &txt, std::vector<double> *out) {
        out->clear();
        const QString t = txt.trimmed();
        if (t.isEmpty()) {
            return;
        }
        const QStringList parts = t.split(QLatin1Char(','), Qt::SkipEmptyParts);
        for (const QString &p : parts) {
            bool ok = false;
            const double v = p.trimmed().toDouble(&ok);
            if (ok && v > 1e-6 && v < 1.0 - 1e-6) {
                out->push_back(v);
            }
        }
        std::sort(out->begin(), out->end());
        out->erase(std::unique(out->begin(), out->end()), out->end());
    };
    parseFracList(m_lineColumnZzBuckling ? m_lineColumnZzBuckling->text() : QString(),
                  &in.columnZzBucklingBraceHeightFractions);
    parseFracList(m_lineColumnLtbBottom ? m_lineColumnLtbBottom->text() : QString(),
                  &in.columnLtbBottomFlangeBraceHeightFractions);
    parseFracList(m_lineColumnLtbTop ? m_lineColumnLtbTop->text() : QString(),
                  &in.columnLtbTopFlangeBraceHeightFractions);
    in.fy_MPa = ui->spin_fy->value();
    in.columnBucklingKy = ui->spin_colBucklingKy->value();
    in.columnBucklingKz = ui->spin_colBucklingKz->value();
    in.columnBucklingCurveOrdinal = ui->combo_colBucklingCurve->currentIndex();
    in.trussBucklingCurveOrdinal = ui->combo_trussBucklingCurve->currentIndex();
    in.columnLateralBraceFromTrussGeometry = ui->check_colBraceFromTruss->isChecked();
    in.columnLateralBraceHeightFractions.clear();
    const QString braceTxt = ui->line_colBraceHeights->text().trimmed();
    if (!braceTxt.isEmpty()) {
        const QStringList parts = braceTxt.split(QLatin1Char(','), Qt::SkipEmptyParts);
        for (const QString &p : parts) {
            bool ok = false;
            const double v = p.trimmed().toDouble(&ok);
            if (ok && v > 1e-6 && v < 1.0 - 1e-6) {
                in.columnLateralBraceHeightFractions.push_back(v);
            }
        }
        std::sort(in.columnLateralBraceHeightFractions.begin(), in.columnLateralBraceHeightFractions.end());
        in.columnLateralBraceHeightFractions.erase(
            std::unique(in.columnLateralBraceHeightFractions.begin(), in.columnLateralBraceHeightFractions.end()),
            in.columnLateralBraceHeightFractions.end());
    }
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

        /** Alt/üst başlık tek 2×L kesiti — küçük segment etiketi yok; döngüden sonra tek büyük etiket. */
        const bool skipSmallChordLabel =
            m.isTruss
            && (m.trussRole == TrussMemberRole::ChordBottom || m.trussRole == TrussMemberRole::ChordTop);

        const QString pLbl = memberProfileLabel(m, opt);
        if (!skipSmallChordLabel && !pLbl.isEmpty() && pLbl != QStringLiteral("—")) {
            const QPointF mid = (p1 + p2) * 0.5;
            QPointF pos = mid;
            const double dx = p2.x() - p1.x();
            const double dy = p2.y() - p1.y();
            const double len = std::hypot(dx, dy);
            if (len > 1e-6) {
                const double ux = dx / len;
                const double uy = dy / len;
                pos += QPointF(-uy, ux) * 9.0;
            }
            auto *ti = m_scene->addSimpleText(pLbl, QFont(QStringLiteral("Segoe UI"), 7, QFont::DemiBold));
            ti->setBrush(QColor(22, 22, 22));
            const QRectF tb = ti->boundingRect();
            ti->setTransformOriginPoint(tb.center());
            ti->setPos(pos.x() - 0.5 * tb.width(), pos.y() - 0.5 * tb.height());
            double deg = 0.0;
            if (len > 1e-6) {
                deg = std::atan2(dy, dx) * 180.0 / 3.14159265358979323846;
                /** Okunabilirlik: metin ters dönmesin (|açı| > 90° ise 180° çevir). */
                if (deg > 90.0) {
                    deg -= 180.0;
                } else if (deg < -90.0) {
                    deg += 180.0;
                }
            }
            ti->setRotation(deg);
            ti->setZValue(5);
        }
    }

    /** Makas alt hat: tek kesit → tek büyük etiket. Üst hat: mahya sol/sağ iki yarı ayrı (tekrar yok, overlap az). */
    const double ridgeX = 0.5 * input.spanWidth_m;
    const double kChordOffPx = 13.0;

    auto addChordGroupLabelFiltered = [&](TrussMemberRole role, const QString &txt,
                                          const std::function<bool(double mx, double my)> &useSegment,
                                          std::optional<bool> topChordLeftHalf) {
        const QString t = stripProfileSuffix(txt);
        if (t.isEmpty() || t == QStringLiteral("—")) {
            return;
        }
        QPointF sumMid(0.0, 0.0);
        double sumDx = 0.0;
        double sumDy = 0.0;
        int nSeg = 0;
        for (const auto &mm : result.members) {
            if (!mm.isTruss || mm.trussRole != role) {
                continue;
            }
            double xa = 0.0;
            double ya = 0.0;
            double xb = 0.0;
            double yb = 0.0;
            if (!nodeXY(mm.nodeI, xa, ya) || !nodeXY(mm.nodeJ, xb, yb)) {
                continue;
            }
            const double mx = 0.5 * (xa + xb);
            const double my = 0.5 * (ya + yb);
            if (!useSegment(mx, my)) {
                continue;
            }
            const QPointF pa = scenePoint(xa, ya, scale, originX, originY);
            const QPointF pb = scenePoint(xb, yb, scale, originX, originY);
            sumMid += (pa + pb) * 0.5;
            sumDx += (pb.x() - pa.x());
            sumDy += (pb.y() - pa.y());
            ++nSeg;
        }
        if (nSeg <= 0) {
            return;
        }
        QPointF pos = sumMid / static_cast<double>(nSeg);
        const double len = std::hypot(sumDx, sumDy);
        QPointF perp(0.0, 0.0);
        if (len > 1e-6) {
            perp = QPointF(-sumDy / len, sumDx / len);
        }
        /** Üst başlık sol/sağ: dik yönü dışarı (sola / sağa) çevir — tepe düğümünde çakışmayı azaltır. */
        if (role == TrussMemberRole::ChordTop && topChordLeftHalf.has_value() && len > 1e-6) {
            if (*topChordLeftHalf) {
                if (perp.x() > 0.0) {
                    perp = -perp;
                }
            } else {
                if (perp.x() < 0.0) {
                    perp = -perp;
                }
            }
        } else if (role == TrussMemberRole::ChordBottom && len > 1e-6) {
            /** Alt hat: etiketi çizginin “dışına” (genelde aşağı / örtüşmeden uzak). */
            if (perp.y() < 0.0) {
                perp = -perp;
            }
        }
        pos += perp * kChordOffPx;
        constexpr int kChordLabelPt = 10;
        auto *ti = m_scene->addSimpleText(t, QFont(QStringLiteral("Segoe UI"), kChordLabelPt, QFont::DemiBold));
        ti->setBrush(QColor(18, 18, 18));
        const QRectF tb = ti->boundingRect();
        ti->setTransformOriginPoint(tb.center());
        ti->setPos(pos.x() - 0.5 * tb.width(), pos.y() - 0.5 * tb.height());
        double deg = 0.0;
        if (len > 1e-6) {
            deg = std::atan2(sumDy, sumDx) * 180.0 / 3.14159265358979323846;
            if (deg > 90.0) {
                deg -= 180.0;
            } else if (deg < -90.0) {
                deg += 180.0;
            }
        }
        ti->setRotation(deg);
        ti->setZValue(6);
    };

    addChordGroupLabelFiltered(TrussMemberRole::ChordBottom, opt.trussBottomChord2xL,
                               [](double, double) { return true; }, std::nullopt);
    addChordGroupLabelFiltered(TrussMemberRole::ChordTop, opt.trussTopChord2xL,
                               [ridgeX](double mx, double) { return mx < ridgeX; }, true);
    addChordGroupLabelFiltered(TrussMemberRole::ChordTop, opt.trussTopChord2xL,
                               [ridgeX](double mx, double) { return mx >= ridgeX; }, false);

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

    m_scene->setSceneRect(m_scene->itemsBoundingRect().adjusted(-10, -10, 10, 10));
}

void MainWindow::onCalculateClicked()
{
    if (m_busy) {
        return;
    }

    ui->label_status->setText(tr("Analiz çalışıyor…"));
    ui->text_result_opensees->setPlainText(tr("…"));
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
        ui->text_result_opensees->setPlainText(r.errorMessage);
        ui->label_status->setText(tr("Hata — ayrıntı altta OpenSees alanında."));
        QMessageBox::warning(this, tr("Analysis"), r.errorMessage);
        return;
    }

    ui->text_result_opensees->setPlainText(r.errorMessage);
    ui->label_status->setText(tr("Tamam — OpenSees özeti altta."));
    drawFrameFromResult(r, collectInput());
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    if (!m_busy) {
        setupScene();
    }
}
