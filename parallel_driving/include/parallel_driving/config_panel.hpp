
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

#include "./qnode.hpp"
#include "ui_config_panel.h"

namespace parallel_driving {

class ConfigPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigPanel(QWidget *parent = 0);
    ~ConfigPanel();

    void stringSplit(const std::string& str, const char split, 
                        std::vector<std::string>& res);

    void addChekoBox(std::string topicName);

Q_SIGNALS:
    void getConfigInfo(ConfigInfo *config);
    // 广播获取节点话题信号 给main_window
    void getTopic_signal(const QString &node);      
    // 用户选择订阅的话题信号
    void getSelectedImg_signal(QStringList *topics);

public Q_SLOTS:
    void ros_connect_clicked();
    void refresh_node();
    void closeConfigPanel();
    void activated_slot(const QString & text);
    
    void getSelectedCheckItems();

private:

    void initWindow();

    Ui::ConfigPanel* ui;

    ros::V_string rosNodes;     // 存储当前网络中的节点名称

    QListWidget *listwidget;

};

}

#endif