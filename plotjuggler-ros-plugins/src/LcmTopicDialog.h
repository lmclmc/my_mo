#ifndef LCMTOPICDIALOG_H
#define LCMTOPICDIALOG_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QHeaderView>
#include <QMessageBox>
#include <QGroupBox>
#include <QGridLayout>
#include <map>

struct TopicLcmItem
{
    QString topic;
    QString port;
    QString lcmFilePath;
};

class LcmTopicDialog : public QDialog
{
    Q_OBJECT
public:
    explicit LcmTopicDialog(int mode, QWidget *parent = nullptr);
    QVector<TopicLcmItem> getConfig() const;
    bool eventFilter(QObject *watched, QEvent *event) override;

public:
    QString getLogSavePath() const { return m_logPathEdit->text().trimmed(); }
    int getLogMaxSize() const { return m_logMaxSizeEdit->text().toInt() * 1024 * 1024; }
    int getLogMaxFiles() const { return m_logMaxFilesEdit->text().toInt(); }

private:
    int m_mode;

    void setupUI();
    void addEmptyRow();
    void delSelectedRow();
    void clearAllRows();
    void selectLcmFileForCell(int row);
    void saveConfig();
    void loadConfig();

    QTableWidget* m_table;
    QPushButton* m_btnAdd;
    QPushButton* m_btnDel;
    QPushButton* m_btnClear;
    QPushButton* m_btnSaveCfg;
    QPushButton* m_btnLoadCfg;
    QPushButton* m_btnConfirm;
    // 日志保存相关控件
    QLineEdit* m_logPathEdit;      // 日志保存路径
    QLineEdit* m_logMaxSizeEdit;   // 单文件最大大小(MB)
    QLineEdit* m_logMaxFilesEdit;  // 最大保存文件数
};

#endif // LCMTOPICDIALOG_H