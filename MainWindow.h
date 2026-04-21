#pragma once

#include <QMainWindow>
#include <QGraphicsScene>
#include <QFutureWatcher>
#include <memory>

class QResizeEvent;
class QComboBox;
class QLineEdit;

#include "PortalSolver.h"

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
    PortalFrameInput collectInput() const;

    std::unique_ptr<Ui::MainWindow> ui;
    QGraphicsScene *m_scene{nullptr};
    QComboBox *m_comboColumnBase{nullptr};
    QLineEdit *m_lineColumnZzBuckling{nullptr};
    QLineEdit *m_lineColumnLtbBottom{nullptr};
    QLineEdit *m_lineColumnLtbTop{nullptr};
    QComboBox *m_comboColumnLtbCurve{nullptr};

    QFutureWatcher<PortalFrameResult> m_watcher;
    bool m_busy{false};

protected:
    void resizeEvent(QResizeEvent *event) override;
};
