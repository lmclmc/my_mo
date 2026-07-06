#include "LcmTopicDialog.h"
#include <QLabel>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QEvent>

// 仿PlotJuggler深色样式
static QString getDarkStyleSheet()
{
    return R"(
    QDialog { background-color: #27282c; color: #eeeeee; }
    QTableWidget { background-color: #323338; border: none; gridline-color: #444444; }
    QTableWidget::item { padding: 4px; }
    QHeaderView::section { background-color: #3c3d42; color: white; padding:6px; border:none; }
    QLineEdit { background:#1e1f22; border:1px solid #555; color:#fff; border-radius:3px; padding:4px; }
    QPushButton { background:#484a50; color:#fff; border:none; border-radius:4px; padding:6px 12px; }
    QPushButton:hover { background:#5c5e66; }
    QPushButton:pressed { background:#3a3c40; }
    QPushButton#btnConfirm { background:#2196F3; color:#fff; font-weight:bold; padding:8px 24px; font-size:14px; }
    QPushButton#btnConfirm:hover { background:#42A5F5; }
    QPushButton#btnConfirm:pressed { background:#1E88E5; }
    )";
}

LcmTopicDialog::LcmTopicDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle("LCM Topic & Log Mapper (PlotJuggler Style)");
    setMinimumSize(720, 480);
    setStyleSheet(getDarkStyleSheet());
    setupUI();
}

void LcmTopicDialog::setupUI()
{
    // 主布局
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12,12,12,12);
    mainLayout->setSpacing(10);

    // 表格控件：2列
    m_table = new QTableWidget();
    m_table->setColumnCount(2);
    m_table->setHorizontalHeaderLabels({"Topic Name", "LCM Log File"});
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

    // 底部按钮栏
    QHBoxLayout* btnLayout = new QHBoxLayout();
    m_btnAdd = new QPushButton("+ Add Row");
    m_btnDel = new QPushButton("- Delete Selected");
    m_btnClear = new QPushButton("Clear All");
    m_btnSaveCfg = new QPushButton("Save Config JSON");
    m_btnLoadCfg = new QPushButton("Load Config JSON");

    btnLayout->addWidget(m_btnAdd);
    btnLayout->addWidget(m_btnDel);
    btnLayout->addWidget(m_btnClear);

    // 确认按钮 - 显眼的蓝色按钮
    m_btnConfirm = new QPushButton("Confirm");
    m_btnConfirm->setObjectName("btnConfirm");  // 设置对象名以便应用样式
    btnLayout->addWidget(m_btnConfirm);
    
    btnLayout->addStretch();
    btnLayout->addWidget(m_btnLoadCfg);
    btnLayout->addWidget(m_btnSaveCfg);

    mainLayout->addWidget(m_table);
    mainLayout->addLayout(btnLayout);

    // 绑定信号槽
    connect(m_btnAdd, &QPushButton::clicked, this, &LcmTopicDialog::addEmptyRow);
    connect(m_btnDel, &QPushButton::clicked, this, &LcmTopicDialog::delSelectedRow);
    connect(m_btnClear, &QPushButton::clicked, this, &LcmTopicDialog::clearAllRows);
    connect(m_btnConfirm, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_btnSaveCfg, &QPushButton::clicked, this, &LcmTopicDialog::saveConfig);
    connect(m_btnLoadCfg, &QPushButton::clicked, this, &LcmTopicDialog::loadConfig);

    // 默认一行
    addEmptyRow();
}

void LcmTopicDialog::addEmptyRow()
{
    int row = m_table->rowCount();
    m_table->insertRow(row);

    // 第一列：输入框
    QLineEdit* topicEdit = new QLineEdit();
    topicEdit->setPlaceholderText("e.g. /joint_state");
    m_table->setCellWidget(row, 0, topicEdit);

    // 第二列：显示文件路径，单击弹窗选择
    QLineEdit* fileEdit = new QLineEdit();
    fileEdit->setPlaceholderText("Click to select .lcm log");
    fileEdit->setReadOnly(true);
    fileEdit->installEventFilter(this);  // 安装事件过滤器
    m_table->setCellWidget(row, 1, fileEdit);
}

void LcmTopicDialog::delSelectedRow()
{
    // 使用 selectedRanges() 获取选中的行范围，处理单元格中有控件的情况
    QSet<int> rows;
    auto ranges = m_table->selectedRanges();
    for(auto& range : ranges) {
        for(int row = range.topRow(); row <= range.bottomRow(); ++row) {
            rows.insert(row);
        }
    }
    QList<int> rowList = rows.values();
    std::sort(rowList.rbegin(), rowList.rend());
    for(int r : rowList) m_table->removeRow(r);
}

void LcmTopicDialog::clearAllRows()
{
    m_table->setRowCount(0);
}

bool LcmTopicDialog::eventFilter(QObject *watched, QEvent *event)
{
    if (event->type() == QEvent::MouseButtonPress) {
        QLineEdit* edit = qobject_cast<QLineEdit*>(watched);
        if (edit) {
            // 找到这个QLineEdit所在的行和列
            for (int row = 0; row < m_table->rowCount(); ++row) {
                if (m_table->cellWidget(row, 1) == edit) {
                    selectLcmFileForCell(row);
                    return true;
                }
            }
        }
    }
    return QDialog::eventFilter(watched, event);
}

void LcmTopicDialog::selectLcmFileForCell(int row)
{
    QString path = QFileDialog::getOpenFileName(this,
        "Select LCM Log File", "", "LCM Log (*.lcm)");
    if(path.isEmpty()) return;

    QLineEdit* edit = qobject_cast<QLineEdit*>(m_table->cellWidget(row, 1));
    if(edit) edit->setText(path);
}

QVector<TopicLcmItem> LcmTopicDialog::getConfig() const
{
    QVector<TopicLcmItem> res;
    int rows = m_table->rowCount();
    for(int r=0; r<rows; r++)
    {
        QLineEdit* topicEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,0));
        QLineEdit* fileEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,1));
        if(!topicEd || !fileEd) continue;

        QString t = topicEd->text().trimmed();
        QString f = fileEd->text().trimmed();
        if(t.isEmpty() && f.isEmpty()) continue;

        res.push_back({t, f});
    }
    return res;
}

void LcmTopicDialog::saveConfig()
{
    QString path = QFileDialog::getSaveFileName(this,
        "Save config", "topic_lcm_map.json", "JSON (*.json)");
    if(path.isEmpty()) return;

    QJsonArray arr;
    auto data = getConfig();
    for(auto& item : data)
    {
        QJsonObject obj;
        obj["topic"] = item.topic;
        obj["lcm_file"] = item.lcmFilePath;
        arr.append(obj);
    }
    QJsonDocument doc(arr);
    QFile f(path);
    if(f.open(QIODevice::WriteOnly))
    {
        f.write(doc.toJson(QJsonDocument::Indented));
        f.close();
        QMessageBox::information(this, "OK", "Config saved");
    }
}

void LcmTopicDialog::loadConfig()
{
    QString path = QFileDialog::getOpenFileName(this,
        "Load config", "", "JSON (*.json)");
    if(path.isEmpty()) return;

    QFile f(path);
    if(!f.open(QIODevice::ReadOnly)) return;
    auto doc = QJsonDocument::fromJson(f.readAll());
    f.close();
    if(!doc.isArray()) return;

    clearAllRows();
    auto arr = doc.array();
    for(auto val : arr)
    {
        if(!val.isObject()) continue;
        auto obj = val.toObject();
        addEmptyRow();
        int r = m_table->rowCount()-1;

        QLineEdit* tEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,0));
        QLineEdit* fEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,1));
        if(tEd) tEd->setText(obj["topic"].toString());
        if(fEd) fEd->setText(obj["lcm_file"].toString());
    }
}
