#ifndef GUI_2_H
#define GUI_2_H


#include <QWidget>
///#include <QtXml/QXmlStreamReader>
#include <QtXml>
#include <QFile>
#include <fstream>
#include <QtCore/QMessageLogger>
#include <iostream>
#include <QApplication>
#include <QXmlStreamReader>
#include <QDomAttr>
#include <string>

namespace Ui {
class gui_2;
}


using namespace std;

class gui_2:  public QWidget
{
    Q_OBJECT

public:
    explicit gui_2(QWidget *parent = 0);
    void readFile();
    void changeGUI();
    ~gui_2();

private slots:
    void on_change_clicked();

    void on_parser_clicked();

    void on_textEdit_textChanged();


    void on_gate_marker2_to_pool_textChanged(const QString &arg1);

private:
    Ui::gui_2 *ui;
    QDomDocument xmlDOC;
    QDomElement root, robot, link_elem, joint_elem, org_elem;
    QDomNode joint_node, link_node, org_node;
    QString root_tag, link_tag, joint_tag, org_tag;
    QFile* file;
    int count;
    QDomNodeList nodlist, botlist, jointlist, nodlist2,orglist;
    QDomAttr attrJoint, attrOrg, attrXYZ, attrRPY, xyz,rpy,joint;
    QDomElement elem, elem2;
    string nameElem1, nameElem2, ch, joint_name ;

};

#endif // GUI_2_H
