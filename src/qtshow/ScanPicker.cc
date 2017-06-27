#include <QFileDialog>

#include "qtshow/ScanPicker.h"


ScanPicker::ScanPicker(const QString& current_directory, QWidget *parent, Qt::WindowFlags f)
  : QDialog(parent, f)
{
  setupUi(this);
  lineEditDirectory->setText(current_directory);
}

void ScanPicker::accept() {
  emit scanPicked(
    lineEditDirectory->text(),
    comboBoxFormat->currentText(),
    spinBoxStart->value(),
    spinBoxEnd->value(),
    spinBoxScale->value()
  );
  QDialog::accept();
}

void ScanPicker::chooseScanDirectory() {
  QString dir = QFileDialog::getExistingDirectory(this, "Choose Scan Directory…",
    lineEditDirectory->text());
  lineEditDirectory->setText(dir);
}
