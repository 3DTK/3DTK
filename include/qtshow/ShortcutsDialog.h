#ifndef __SHORTCUTSDIALOG_H__
#define __SHORTCUTSDIALOG_H__

#include "ui_ShortcutsDialog.h"

class ShortcutsDialog : public QDialog, public Ui::ShortcutsDialog {
  Q_OBJECT

public:
  ShortcutsDialog(QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
};

#endif
