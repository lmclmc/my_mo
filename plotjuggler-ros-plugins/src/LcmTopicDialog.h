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
#include <map>

struct TopicLcmItem
{
    QString topic;
    QString lcmFilePath;
};

class LcmTopicDialog : public QDialog
{
    Q_OBJECT
public:
    explicit LcmTopicDialog(QWidget *parent = nullptr);
    QVector<TopicLcmItem> getConfig() const;
    bool eventFilter(QObject *watched, QEvent *event) override;

public:
    QString getPort() const { return m_portEdit->text().trimmed(); }

private:
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
    QLineEdit* m_portEdit;  // 端口输入框
};

#endif // LCMTOPICDIALOG_H