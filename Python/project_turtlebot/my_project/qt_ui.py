# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'qt.ui'
##
## Created by: Qt User Interface Compiler version 6.10.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QLabel, QMainWindow,
    QPushButton, QSizePolicy, QStackedWidget, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(653, 510)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.stackedWidget = QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.stackedWidget.setGeometry(QRect(-71, -10, 751, 521))
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.RIGHT = QPushButton(self.page)
        self.RIGHT.setObjectName(u"RIGHT")
        self.RIGHT.setGeometry(QRect(330, 260, 71, 71))
        self.GO = QPushButton(self.page)
        self.GO.setObjectName(u"GO")
        self.GO.setGeometry(QRect(250, 180, 71, 71))
        self.LEFT = QPushButton(self.page)
        self.LEFT.setObjectName(u"LEFT")
        self.LEFT.setGeometry(QRect(170, 260, 71, 71))
        self.AUTO = QPushButton(self.page)
        self.AUTO.setObjectName(u"AUTO")
        self.AUTO.setGeometry(QRect(470, 260, 121, 71))
        self.BACK = QPushButton(self.page)
        self.BACK.setObjectName(u"BACK")
        self.BACK.setGeometry(QRect(250, 340, 71, 71))
        self.STOP = QPushButton(self.page)
        self.STOP.setObjectName(u"STOP")
        self.STOP.setGeometry(QRect(250, 260, 71, 71))
        self.stackedWidget.addWidget(self.page)
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
        self.TO_PAGE_1 = QPushButton(self.page_2)
        self.TO_PAGE_1.setObjectName(u"TO_PAGE_1")
        self.TO_PAGE_1.setGeometry(QRect(90, 20, 91, 51))
        self.START = QPushButton(self.page_2)
        self.START.setObjectName(u"START")
        self.START.setGeometry(QRect(510, 20, 91, 51))
        self.CANCEL = QPushButton(self.page_2)
        self.CANCEL.setObjectName(u"CANCEL")
        self.CANCEL.setGeometry(QRect(610, 20, 91, 51))
        self.frame = QFrame(self.page_2)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(190, 80, 400, 400))
        self.frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame.setFrameShadow(QFrame.Shadow.Raised)
        self.STATE = QPushButton(self.page_2)
        self.STATE.setObjectName(u"STATE")
        self.STATE.setGeometry(QRect(410, 20, 91, 51))
        self.LABEL = QLabel(self.page_2)
        self.LABEL.setObjectName(u"LABEL")
        self.LABEL.setGeometry(QRect(246, 26, 151, 41))
        self.stackedWidget.addWidget(self.page_2)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.RIGHT.setText(QCoreApplication.translate("MainWindow", u"RIGHT", None))
        self.GO.setText(QCoreApplication.translate("MainWindow", u"GO", None))
        self.LEFT.setText(QCoreApplication.translate("MainWindow", u"LEFT", None))
        self.AUTO.setText(QCoreApplication.translate("MainWindow", u"AUTO-SUMMON", None))
        self.BACK.setText(QCoreApplication.translate("MainWindow", u"BACK", None))
        self.STOP.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.TO_PAGE_1.setText(QCoreApplication.translate("MainWindow", u"<-", None))
        self.START.setText(QCoreApplication.translate("MainWindow", u"START", None))
        self.CANCEL.setText(QCoreApplication.translate("MainWindow", u"CANCEL", None))
        self.STATE.setText(QCoreApplication.translate("MainWindow", u"STATE", None))
        self.LABEL.setText("")
    # retranslateUi

