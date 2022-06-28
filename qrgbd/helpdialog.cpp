#include "helpdialog.h"
#include "QFile"
#include "ui_helpdialog.h"

HelpDialog::HelpDialog(QWidget *parent) : QDialog(parent),
                                          ui(new Ui::HelpDialog) {
    ui->setupUi(this);
    this->setWindowTitle("Help Document");
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        this->close();
    });
    // set "QRGBD" images
    ui->label_Q->setPixmap(QPixmap("../img/q.png").scaledToWidth(ui->label_Q->width()));
    ui->label_R->setPixmap(QPixmap("../img/r.png").scaledToWidth(ui->label_R->width()));
    ui->label_G->setPixmap(QPixmap("../img/g.png").scaledToWidth(ui->label_G->width()));
    ui->label_B->setPixmap(QPixmap("../img/b.png").scaledToWidth(ui->label_B->width()));
    ui->label_D->setPixmap(QPixmap("../img/d.png").scaledToWidth(ui->label_D->width()));
}

HelpDialog::~HelpDialog() {
    delete ui;
}
