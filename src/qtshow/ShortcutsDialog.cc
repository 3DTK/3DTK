#include <QDialog>

#include "qtshow/ShortcutsDialog.h"

ShortcutsDialog::ShortcutsDialog(QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
{
  setupUi(this);
}
