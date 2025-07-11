// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QtGui>
#include <QApplication>
#include "open_manipulator_x_gui/main_window.hpp"
#include "open_manipulator_x_gui/qnode.hpp"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  open_manipulator_x_gui::MainWindow w(argc, argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

  int result = app.exec();
  return result;
}
