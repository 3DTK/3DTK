#include <QDialog>

#include "qtshow/SettingsDialog.h"


SettingsDialog::SettingsDialog(QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
{
  setupUi(this);
}
