#include "selfdrive/ui/qt/widgets/wifi.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>

WiFiPromptWidget::WiFiPromptWidget(QWidget *parent) : QFrame(parent) {
  setObjectName("primeWidget");
  // Setup Installation Guide
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(56, 40, 56, 40);
  main_layout->setSpacing(42);

  QLabel *title = new QLabel(tr("Installation Guide"));
  title->setStyleSheet("font-size: 64px; font-weight: 500;");
  main_layout->addWidget(title);

  QLabel *desc = new QLabel(tr("Click below to get more information about us"));
  desc->setStyleSheet("font-size: 40px; font-weight: 400;");
  desc->setWordWrap(true);
  main_layout->addWidget(desc);

  main_layout->addStretch();

  QPushButton *settings_btn = new QPushButton(tr("QR Code"));
  connect(settings_btn, &QPushButton::clicked, [=]() { emit openInstallationGuide(); });
  settings_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 55px;
      font-weight: 500;
      border-radius: 10px;
      background-color: #465BEA;
      padding: 64px;
    }
    QPushButton:pressed {
      background-color: #3049F4;
    }
  )");
  main_layout->addWidget(settings_btn);

  setStyleSheet(R"(
    #primeWidget {
      background-color: #333333;
      border-radius: 10px;
    }
  )");
}