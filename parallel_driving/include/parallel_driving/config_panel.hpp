
#ifndef parallel_driving_CONFIG_PANEL_HPP_
#define parallel_driving_CONFIG_PANEL_HPP_

#include <QWidget>
#include <QString>

#include "./qnode.hpp"
#include "ui_config_panel.h"

namespace parallel_driving {

class ConfigPanel : public QWidget
{
    Q_OBJECT

public:
    explicit ConfigPanel(QWidget *parent = 0);
    ~ConfigPanel();

Q_SIGNALS:
    void getConfigInfo(ConfigInfo *config);

public Q_SLOTS:
    void ros_connect_clicked();

private:

    void initWindow();

    Ui::ConfigPanel* ui;
};

}

#endif