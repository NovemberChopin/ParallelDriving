#ifndef parallel_driving_PAGE_LEFT_HPP_
#define parallel_driving_PAGE_LEFT_HPP_

#include <QWidget>
#include <QString>

#include "ui_page_left.h"

namespace parallel_driving {


class PageLeft : public QWidget
{
    Q_OBJECT

public:
    explicit PageLeft(QWidget *parent = 0);
    ~PageLeft();


Q_SIGNALS:


private:

    Ui::PageLeft* ui;
};

}

#endif