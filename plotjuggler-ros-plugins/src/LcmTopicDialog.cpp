#include "LcmTopicDialog.h"
#include <QLabel>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QEvent>

// 仿PlotJuggler深色样式
static QString getDarkStyleSheet()
{
    // return R"(
    // QDialog { background-color: #27282c; color: #eeeeee; }
    // QTableWidget { background-color: #323338; border: none; gridline-color: #444444; }
    // QTableWidget::item { padding: 4px; }
    // QHeaderView::section { background-color: #3c3d42; color: white; padding:6px; border:none; }
    // QLineEdit { background:#1e1f22; border:1px solid #555; color:#fff; border-radius:3px; padding:4px; }
    // QPushButton { background:#484a50; color:#fff; border:none; border-radius:4px; padding:6px 12px; }
    // QPushButton:hover { background:#5c5e66; }
    // QPushButton:pressed { background:#3a3c40; }
    // QPushButton#btnConfirm { background:#2196F3; color:#fff; font-weight:bold; padding:8px 24px; font-size:14px; }
    // QPushButton#btnConfirm:hover { background:#42A5F5; }
    // QPushButton#btnConfirm:pressed { background:#1E88E5; }
    // )";

    return R"(
    QTableWidget::item { padding: 4px; }
    QPushButton:pressed { background:#3a3c40; }
    QPushButton#btnConfirm:pressed { background:#1E88E5; }
    )";
}

LcmTopicDialog::LcmTopicDialog(int mode, QWidget *parent)
    : QDialog(parent), m_mode(mode)
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

    if (m_mode == 1) {
        // 日志保存设置区域
        QGroupBox* logGroup = new QGroupBox("LCM Log Save Settings");
        logGroup->setStyleSheet("QGroupBox { font-weight: bold; border: 1px solid #555; border-radius: 4px; padding: 8px; margin-top: 8px; } QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px 0 4px; }");
        QGridLayout* logLayout = new QGridLayout(logGroup);
        logLayout->setSpacing(8);

        // 日志保存路径
        QLabel* logPathLabel = new QLabel("Log Save Path:");
        logPathLabel->setStyleSheet("font-size: 13px;");
        m_logPathEdit = new QLineEdit();
        m_logPathEdit->setPlaceholderText("Click to select save directory");
        m_logPathEdit->setReadOnly(true);
        m_logPathEdit->installEventFilter(this);
        QPushButton* logPathBtn = new QPushButton("Browse");
        logPathBtn->setFixedWidth(60);
        connect(logPathBtn, &QPushButton::clicked, this, [this]() {
            QString dir = QFileDialog::getSaveFileName(
                this, 
                "Select Log Save Directory");
            if (!dir.isEmpty()) {
                m_logPathEdit->setText(dir);
                qDebug() << "Log save directory selected:" << dir;
            }
        });

        // 单文件最大大小
        QLabel* logSizeLabel = new QLabel("Max File Size (MB):");
        logSizeLabel->setStyleSheet("font-size: 13px;");
        m_logMaxSizeEdit = new QLineEdit();
        m_logMaxSizeEdit->setPlaceholderText("e.g. 100");
        m_logMaxSizeEdit->setFixedWidth(80);
        m_logMaxSizeEdit->setText("100");  // 默认值

        // 最大保存文件数
        QLabel* logFilesLabel = new QLabel("Max Files:");
        logFilesLabel->setStyleSheet("font-size: 13px;");
        m_logMaxFilesEdit = new QLineEdit();
        m_logMaxFilesEdit->setPlaceholderText("e.g. 10");
        m_logMaxFilesEdit->setFixedWidth(80);
        m_logMaxFilesEdit->setText("10");  // 默认值

        logLayout->addWidget(logPathLabel, 0, 0);
        logLayout->addWidget(m_logPathEdit, 0, 1);
        logLayout->addWidget(logPathBtn, 0, 2);
        logLayout->addWidget(logSizeLabel, 1, 0);
        logLayout->addWidget(m_logMaxSizeEdit, 1, 1);
        logLayout->addWidget(logFilesLabel, 1, 2);
        logLayout->addWidget(m_logMaxFilesEdit, 1, 3);

        mainLayout->addWidget(logGroup);
    }

    // 表格控件：3列（Topic Name, Port, LCM Log File）
    m_table = new QTableWidget();
    m_table->setColumnCount(3);
    m_table->setHorizontalHeaderLabels({"Topic Name", "Port", "LCM Log File"});
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    m_table->setColumnWidth(0, 220);  // Topic Name 列固定宽度220像素
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    m_table->setColumnWidth(1, 120);  // Port 列固定宽度120像素
    m_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);  // LCM Log File 拉伸填充

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

    // 作者信息标签 - 高亮显示并支持链接跳转
    QLabel* authorLabel = new QLabel();
    authorLabel->setStyleSheet("font-size: 12px; color: #888888; padding: 4px;");
    authorLabel->setAlignment(Qt::AlignLeft);
    authorLabel->setOpenExternalLinks(true);  // 允许打开外部链接
    authorLabel->setText(QString("作者：<font color='#42A5F5'><b>罗敏成</b></font> | \
                                  邮箱：<a href='mailto:luomincheng@dreame.tech'><font color='#2935a1'><b>luomincheng@dreame.tech</b></font></a> | \
                                  文档：<a href='https://dreametech.feishu.cn/wiki/DiyKw9y6mikuEykygWyc0AZvnSO'><font color='#FFB74D'><b>Wiki</b></font></a> | \
                                  仓库：<a href='https://gitlab.magiclab.top/dreamebiped/controlsoftware/motionstudio'><font color='#AED581'><b>GitLab</b></font></a>"));
    mainLayout->addWidget(authorLabel);

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

    // 第一列：Topic输入框
    QLineEdit* topicEdit = new QLineEdit();
    topicEdit->setPlaceholderText("e.g. /joint_state");
    m_table->setCellWidget(row, 0, topicEdit);

    // 第二列：端口输入框
    QLineEdit* portEdit = new QLineEdit();
    portEdit->setPlaceholderText("e.g. 7777");
    portEdit->setFixedWidth(110);
    m_table->setCellWidget(row, 1, portEdit);

    // 第三列：显示文件路径，单击弹窗选择
    QLineEdit* fileEdit = new QLineEdit();
    fileEdit->setPlaceholderText("Click to select .lcm log");
    fileEdit->setReadOnly(true);
    fileEdit->installEventFilter(this);  // 安装事件过滤器
    m_table->setCellWidget(row, 2, fileEdit);
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
                if (m_table->cellWidget(row, 2) == edit) {
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
        QLineEdit* portEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,1));
        QLineEdit* fileEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,2));
        if(!topicEd || !portEd || !fileEd) continue;

        QString t = topicEd->text().trimmed();
        QString p = portEd->text().trimmed();
        QString f = fileEd->text().trimmed();
        if(t.isEmpty() && p.isEmpty() && f.isEmpty()) continue;

        res.push_back({t, p, f});
    }
    return res;
}

void LcmTopicDialog::saveConfig() {
    QString path = QFileDialog::getSaveFileName(this,
        "Save config", "topic_lcm_map.json", "JSON (*.json)");
    if(path.isEmpty()) return;

    QJsonObject rootObj;

    // 保存 LCM log 配置参数
    rootObj["log_save_path"] = m_logPathEdit->text().trimmed();
    rootObj["log_max_size_mb"] = m_logMaxSizeEdit->text().trimmed();
    rootObj["log_max_files"] = m_logMaxFilesEdit->text().trimmed();

    // 保存 topic 配置
    QJsonArray arr;
    auto data = getConfig();
    for(auto& item : data)
    {
        QJsonObject obj;
        obj["topic"] = item.topic;
        obj["port"] = item.port;
        obj["lcm_file"] = item.lcmFilePath;
        arr.append(obj);
    }
    rootObj["topics"] = arr;

    QJsonDocument doc(rootObj);
    QFile f(path);
    if(f.open(QIODevice::WriteOnly))
    {
        f.write(doc.toJson(QJsonDocument::Indented));
        f.close();
        QMessageBox::information(this, "OK", "Config saved");
    }
}

void LcmTopicDialog::loadConfig() {
    QString path = QFileDialog::getOpenFileName(this,
        "Load config", "", "JSON (*.json)");
    if(path.isEmpty()) return;

    QFile f(path);
    if(!f.open(QIODevice::ReadOnly)) return;
    auto doc = QJsonDocument::fromJson(f.readAll());
    f.close();
    if(!doc.isObject()) return;

    QJsonObject rootObj = doc.object();

    // 加载 LCM log 配置参数
    if(rootObj.contains("log_save_path")) {
        m_logPathEdit->setText(rootObj["log_save_path"].toString());
    }
    if(rootObj.contains("log_max_size_mb")) {
        m_logMaxSizeEdit->setText(rootObj["log_max_size_mb"].toString());
    }
    if(rootObj.contains("log_max_files")) {
        m_logMaxFilesEdit->setText(rootObj["log_max_files"].toString());
    }

    // 加载 topic 配置
    if(!rootObj.contains("topics")) return;
    auto arr = rootObj["topics"].toArray();
    if(arr.isEmpty()) return;

    clearAllRows();
    for(auto val : arr)
    {
        if(!val.isObject()) continue;
        auto obj = val.toObject();
        addEmptyRow();
        int r = m_table->rowCount()-1;

        QLineEdit* tEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,0));
        QLineEdit* pEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,1));
        QLineEdit* fEd = qobject_cast<QLineEdit*>(m_table->cellWidget(r,2));
        if(tEd) tEd->setText(obj["topic"].toString());
        if(pEd) pEd->setText(obj["port"].toString());
        if(fEd) fEd->setText(obj["lcm_file"].toString());
    }
}