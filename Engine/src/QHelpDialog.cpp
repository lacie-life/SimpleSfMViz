#include "QHelpDialog.h"
#include "ui_qhelpdialog.h"

QHelpDialog::QHelpDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QHelpDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Help Document");
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        this->close();
    });

    ui->label_Q->setPixmap(QPixmap("../img/q.png").scaledToWidth(ui->label_Q->width()));
    ui->label_R->setPixmap(QPixmap("../img/r.png").scaledToWidth(ui->label_R->width()));
    ui->label_G->setPixmap(QPixmap("../img/g.png").scaledToWidth(ui->label_G->width()));
    ui->label_B->setPixmap(QPixmap("../img/b.png").scaledToWidth(ui->label_B->width()));
    ui->label_D->setPixmap(QPixmap("../img/d.png").scaledToWidth(ui->label_D->width()));
}

QHelpDialog::~QHelpDialog()
{
    delete ui;
}
