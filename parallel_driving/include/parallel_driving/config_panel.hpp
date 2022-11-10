
#ifndef parallel_driving_CONFIG_PANEL_HPP_
#define parallel_driving_CONFIG_PANEL_HPP_

#include <QWidget>
#include <QString>
#include <QComboBox>
#include <vector>
#include <QCheckBox>
#include <QListWidget>
#include <QListWidgetItem>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QDebug>
#include <unordered_map>
#include "./qnode.hpp"
#include "ui_config_panel.h"

namespace parallel_driving {


struct NodeInfo {
    std::string nodeName;
    ros::master::V_TopicInfo topicInfo;
};


class ConfigPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigPanel(QWidget *parent = 0);
    ~ConfigPanel();

    void stringSplit(const std::string& str, const char split, 
                        std::vector<std::string>& res);

    void addChekoBox(std::string topicName);

    void createNodeMap(std::string nodeName, std::string prefix);

    bool isROSCarNode(std::string nodeName, std::string &prefix);

Q_SIGNALS:
    void getConfigInfo(ConfigInfo *config);      
    // 用户选择订阅的话题信号
    void getSelectedImg_signal(QStringList *topics, std::string prefix);

public Q_SLOTS:
    void ros_connect_clicked();
    void refresh_node();
    void activated_slot(const QString & text);
    
    void getSelectedCheckItems();

private:

    void initWindow();

    Ui::ConfigPanel* ui;
    
    std::unordered_map<std::string, NodeInfo>  node_map;   // 保存所有节点的信息
    ros::master::V_TopicInfo topics_info;
    ros::V_string rosNodes;     // 存储当前网络中的节点名称

    QListWidget *listwidget;
    std::string prefix = "";

};

}

#endif