#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QFutureWatcher>
#include <memory>

class QResizeEvent;
class QComboBox;
class QLineEdit;

#include "PortalSolver.h"

#include <optional>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onCalculateClicked();
    void onAnalysisFinished();

private:
    void setupScene();
    void drawFrameFromResult(const PortalFrameResult &result, const PortalFrameInput &input);
    PortalFrameInput collectInput();
    void updateCharLineLoadsReadout(const PortalFrameInput &in);

    std::unique_ptr<Ui::MainWindow> ui;
    QGraphicsScene *m_scene{nullptr};
    QComboBox *m_comboColumnBase{nullptr};
    QComboBox *m_comboTrussSectionForm{nullptr};
    QLineEdit *m_lineColumnZzBuckling{nullptr};
    QLineEdit *m_lineColumnLtbBottom{nullptr};
    QLineEdit *m_lineColumnLtbTop{nullptr};
    QLineEdit *m_lineTrussRestraints{nullptr};
    QComboBox *m_comboColumnLtbCurve{nullptr};

    QFutureWatcher<PortalFrameResult> m_watcher;
    bool m_busy{false};
    /** Hesapla’ya basıldığında kuyruğa alınan giriş (tam zarf OpenSees bu veriyle çalışır). */
    PortalFrameInput m_inputQueuedForLastAnalyzeSubmit{};
    /** Son başarılı analizde kullanılan giriş; yük/ölçü değişince `result.sectionDesign` kullanılmaz. */
    std::optional<PortalFrameInput> m_inputAtLastSuccessfulAnalyze;
    /** Son başarılı OpenSees çıktısı — form hâlâ bu analizle aynıysa önizlemede boş kuvvet yerine bunu çizer. */
    std::optional<PortalFrameResult> m_lastSuccessfulResult;

protected:
    void resizeEvent(QResizeEvent *event) override;
};
