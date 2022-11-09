
#include <QDebug>
#include <QString>
#include <QPushButton>
#include <QDesktopWidget>
#include "../include/parallel_driving/config_panel.hpp"


using namespace std;

namespace parallel_driving {

ConfigPanel::ConfigPanel(QWidget *parent) :
    QWidget(parent)
    // ui(new ConfigPanel)
{
    ui = new Ui::ConfigPanel();
    ui->setupUi(this);

    QObject::connect(ui->btn_connect, &QPushButton::clicked, this, &ConfigPanel::ros_connect_clicked);
    QObject::connect(ui->btn_refresh, &QPushButton::clicked, this, &ConfigPanel::refresh_node);
    QObject::connect(ui->btn_return, &QPushButton::clicked, this, &ConfigPanel::closeConfigPanel);
    QObject::connect(ui->btn_confirm, &QPushButton::clicked, this, &ConfigPanel::getSelectedCheckItems);
    QObject::connect(ui->comboBox, SIGNAL(activated(const QString)), 
                    this, SLOT(activated_slot(const QString)));
    ui->rosMasterUri->setText("http://192.168.50.23:11311");
    ui->localhost->setText("http://192.168.50.23");
    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {

    // 设置窗体居中显示，并且不能更改大小
    this->setFixedSize(600, 500);
    QDesktopWidget desktop;
    int screenX=desktop.availableGeometry().width();
    int screenY=desktop.availableGeometry().height();
    int winX = this->width();
    int winY = this->height();
    QPoint movePoint(screenX/2 - winX/2, screenY/2 - winY/2);
    this->move(movePoint);

    //设置窗体阻塞所有窗体
    setWindowModality(Qt::WindowModality::ApplicationModal);
    //设置窗体标题栏为没有按钮
    // setWindowButtonType(QBasePara::TypeDialog::None_Dialog);
    //设置窗体关闭后是否释放内存
	setAttribute(Qt::WA_DeleteOnClose, false);
    // ui->ros_connect->setStyleSheet(" border: 2px solid #000000");    

    listwidget = new QListWidget;
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(listwidget);
    ui->showTopics->setLayout(layout);
}


void ConfigPanel::addChekoBox(std::string topicName) {
    QListWidgetItem *item = new QListWidgetItem(listwidget);
    //在当前行添加item  checkbox
    QCheckBox *checkbox = new QCheckBox;
    checkbox->setText(QString::fromStdString(topicName));
    listwidget->addItem(item);
    listwidget->setItemWidget(item,checkbox);
}

/**
 * @brief 选择话题后确认按钮的槽函数
 * 获取选择的图片话题，并发送到qnode
 * 
 */
void ConfigPanel::getSelectedCheckItems() {
    QStringList *itemList = new QStringList();
    for (int i=0; i<listwidget->count(); i++) {
        QListWidgetItem *item = listwidget->item(i);
        //将QWidget 转化为QCheckBox  获取第i个item 的控件
        QCheckBox *checkbox = static_cast<QCheckBox *>(listwidget->itemWidget(item));
        if (checkbox->isChecked()) {
            QString checkboxStr = checkbox->text();
            itemList->append(checkboxStr);
        }
    }
    qDebug() << "------ 选择的话题列表为 -----";
    qDebug() << *itemList;
    if (itemList->size() == 0) {
        qDebug() << "请选择要订阅的话题";
        QMessageBox::information(this, "注意", "请选择话题！");
    } else {
        // 把选择的话题传递给 qnode
        Q_EMIT getSelectedImg_signal(itemList);
    }
    
    this->close();
}



/**
 * @brief 启动 Master 节点
 * 
 */
void ConfigPanel::ros_connect_clicked() {
    ConfigInfo *configInfo = new ConfigInfo();
    configInfo->rosMasterUri = ui->rosMasterUri->text();
    configInfo->localhost = ui->localhost->text();

    // for (int i=0; i<5; i++) {
    //     configInfo->imageTopics.push_back("/hik_cam_node/hik_camera");
    // }

    Q_EMIT getConfigInfo(configInfo);
    
    // this->close();
}


/**
 * @brief 分割字符串
 * 
 * @param str 需要分割的字符串
 * @param split 分割的标志位（字符）
 * @param res   分割的结果
 */
void ConfigPanel::stringSplit(const std::string& str, 
                        const char split, std::vector<std::string>& res)
{
    if (str == "")        return;
    //在字符串末尾也加入分隔符，方便截取最后一段
    string strs = str + split;
    size_t pos = strs.find(split);
  
    // 若找不到内容则字符串搜索函数返回 npos
    while (pos != strs.npos)
    {
        string temp = strs.substr(0, pos);
        res.push_back(temp);
        //去掉已分割的字符串,在剩下的字符串中进行分割
        strs = strs.substr(pos + 1, strs.size());
        pos = strs.find(split);
    }
}


/**
 * @brief 点击刷新节点按钮槽函数
 * 
 */
void ConfigPanel::refresh_node() {
    ui->comboBox->clear();
    this->rosNodes.clear();
    ros::master::getNodes(this->rosNodes);
    std::vector<std::string> strList;
    for (auto node: this->rosNodes) {
        strList.clear();
        stringSplit(node, '_', strList);
        if (strList[0] == "/yhs")
            ui->comboBox->addItem(QString::fromStdString(node));
    }
}


void ConfigPanel::closeConfigPanel() {
    this->close();
}


/**
 * @brief 选择下拉节点列表的槽函数
 * 当选择一个节点名后：
 *      1. 获取后缀，车辆标识
 *      2. 向 main_window 发信号获取该节点发布的话题
 * @param text 
 */
void ConfigPanel::activated_slot(const QString & text) {
    listwidget->clear();
    qDebug() << "select item is: " << text;
    // 广播获取节点话题信号 给main_window
    Q_EMIT getTopic_signal(text);
}


}       // namespace
