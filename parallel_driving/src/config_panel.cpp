
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
    QObject::connect(ui->btn_confirm, &QPushButton::clicked, this, &ConfigPanel::getSelectedCheckItems);
    QObject::connect(ui->comboBox, SIGNAL(activated(const QString)), 
                    this, SLOT(activated_slot(const QString)));
    ui->rosMasterUri->setText("http://127.0.0.1:11311");
    ui->localhost->setText("127.0.0.1");
    ui->rosHostname->setText("parallel_driving");
    initWindow();
}

ConfigPanel::~ConfigPanel()
{
    delete ui;
}


void ConfigPanel::initWindow() {

    // 设置窗体居中显示，并且不能更改大小
    this->setFixedSize(600, 500);
    this->setWindowTitle("车辆配置");
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
    listwidget->setStyleSheet("background-color: rgb(208, 247, 255);");
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
    if (!ros::master::check()) {
        QMessageBox::information(this, "提示", "请先连接主节点!");
    }
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
    // qDebug() << "------ 选择的话题列表为 -----";
    // qDebug() << *itemList;
    if (itemList->size() == 0) {
        QMessageBox::information(this, "注意", "请选择话题！");
    } else if (itemList->size() > 5) {
        QMessageBox::information(this, "注意", "只能选择5个话题!");
    } else {
        int len = itemList->size();
        // 不足五个话题，自动用最后一个话题补全
        QString lastTopic = itemList->at(len-1);
        for (int i=0; i<(5-len); i++) {
            itemList->append(lastTopic);
        }
        // 把选择的话题传递给 qnode
        Q_EMIT getSelectedImg_signal(itemList, this->prefix);
        this->close();
    }
}



/**
 * @brief 启动 Master 节点
 * 
 */
void ConfigPanel::ros_connect_clicked() {
    ConfigInfo *configInfo = new ConfigInfo();
    configInfo->rosMasterUri = ui->rosMasterUri->text();
    configInfo->localhost = ui->localhost->text();
    configInfo->nodename = ui->rosHostname->text();

    Q_EMIT getConfigInfo(configInfo);
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
 * @brief Create a Node Map object
 * 如果 prefix=car2 那么需要找 /car2/hik_cam_node_1/camera_info 这样的图像话题节点
 * @param nodeName 节点名
 * @param prefix 节点后缀，用于标示小车
 */
void ConfigPanel::createNodeMap(std::string nodeName, std::string prefix) {
    std::cout << "nodeName: " << nodeName << "prefix: " << prefix << std::endl;
    NodeInfo nodeinfo;
    nodeinfo.nodeName = nodeName;
    for (auto item: this->topics_info) {
        // std::cout << item.name << " " << item.datatype << std::endl;
        // 以 '/' 分割话题，判断是否为相机话题
        std::vector<std::string> strList;
        stringSplit(item.name, '/', strList);
        
        if (strList[1] == prefix && strList.size() > 3) {
            if (strList[3] == "hik_camera") {
                nodeinfo.topicInfo.push_back(item);
            }
        }
    }
    // 存入哈希表    
    this->node_map.insert({prefix, nodeinfo});

    // unorder_map 查找与遍历
    // auto it = node_map.find("car0");
    // for (auto item: it->second.topicInfo) {
    //     std::cout << item.name << " " << item.datatype << std::endl;
    // }
    // for (auto &a: node_map) {
    //     std::cout << a.first << std::endl;
    //     std::cout << a.second.nodeName << std::endl;
    //     for (auto item: a.second.topicInfo) {
    //         std::cout << item.name << " " << item.datatype << std::endl;
    //     }
    // }
} 

/**
 * @brief 判断一个节点名是否为小车yhs的ROS节点
 * e.g.: /car2/yhs_can_control_node
 * 
 * @param nodeName 
 * @param prefix 
 * @return true 
 * @return false 
 */
bool ConfigPanel::isROSCarNode(std::string nodeName, std::string &prefix) {
    std::vector<std::string> strList;
    this->stringSplit(nodeName, '/', strList);
    if (strList.size() < 3) {
        return false;
    } else {
        std::string sub_name = strList[2];  // yhs_can_control_node
        std::vector<std::string> sub_list;
        this->stringSplit(sub_name, '_', sub_list);
        if (sub_list.size() != 0 && (sub_list[0] == "yhs" || sub_list[0] == "test")) {
            prefix = strList[1];    // car2
            return true;
        } else {
            return false;
        }
    }
}


/**
 * @brief 点击刷新节点按钮槽函数
 * 
 */
void ConfigPanel::refresh_node() {
    if (!ros::master::check())
        return;
    ui->comboBox->clear();
    this->rosNodes.clear();
    this->topics_info.clear();
    // 获取网络下所有节点名
    ros::master::getNodes(this->rosNodes);
    // 获取所有节点发布的消息
    ros::master::getTopics(this->topics_info);

    std::vector<std::string> strList;
    std::vector<std::string>::iterator it;
    int car_num = 0;

    for (auto node: this->rosNodes) {
        std::string prefix = "";
        if (isROSCarNode(node, prefix)) {
            if (node_map.find(prefix) == node_map.end()) {  
                // 如果当前 node_map 中不存在 prefix
                createNodeMap(node, prefix);
            }
            car_num++;
            ui->comboBox->addItem(QString::fromStdString(node));
        }       
    }
    if (car_num == 0) {
        QMessageBox::information(this, "提示", "检查小车节点是否启动！");
    }
}


/**
 * @brief 选择下拉节点列表的槽函数
 * 当选择一个节点名后：
 *      // /car1/yhs_can_control_node
 *      1. 获取后缀，车辆标识
 *      2. 根据标示，展示相应的图像话题
 * @param text 
 */
void ConfigPanel::activated_slot(const QString & text) {
    listwidget->clear();        // 首先清空话题展示页面
    std::vector<std::string> strList;
    this->stringSplit(text.toStdString(), '/', strList);
    
    this->prefix = strList[1];
    std::cout << "get prefix: " << this->prefix << std::endl;  
    auto it_node = this->node_map.find(this->prefix);
    if (it_node != node_map.end())
        for (auto item: it_node->second.topicInfo) {
            this->addChekoBox(item.name);
        }
}


}       // namespace
