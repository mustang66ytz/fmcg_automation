#ifndef SIMPLE_GUI_PANEL_H
#define SIMPLE_GUI_PANEL_H

#include "rviz/panel.h"

namespace rviz_simple_gui
{

// Forward declare blend widget
class SimpleWidget;

class SimplePanel : public rviz::Panel
{
  Q_OBJECT
public:
  SimplePanel(QWidget* parent = 0);

  virtual ~SimplePanel();

  virtual void onInitialize();

protected:
  SimpleWidget* widget_;
};
}

#endif // SIMPLE_GUI_PANEL_H
