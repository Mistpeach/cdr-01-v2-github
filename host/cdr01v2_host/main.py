# -*- coding: utf-8 -*-
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Mistpeach

"""CDR01 v2 上位机入口。"""

import sys

from PySide6.QtWidgets import QApplication

from .ui_main import HostController, MainWindow


def main():
    app = QApplication(sys.argv)
    controller = HostController()
    window = MainWindow(controller)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
