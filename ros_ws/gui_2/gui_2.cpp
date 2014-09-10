#include "gui_2.h"
#include "ui_gui_2.h"



gui_2::gui_2(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::gui_2), count(1)
{
    ui->setupUi(this);
    file = new QFile("/home/manvi/ros_ws/gui_2/scene.urdf");
}

gui_2::~gui_2()
{
    delete ui;
}



void gui_2::readFile(){

    QDomNodeList nodList;
    if(!root.isNull()){
        root_tag = root.tagName();
        string root_name = root_tag.toUtf8().constData();
        botlist = root.childNodes();

        for(int i=0; i<botlist.size(); i++){
            elem = botlist.at(i).toElement();
            attrJoint = elem.attributeNode("name");
            nameElem1 = elem.tagName().toUtf8().constData();
            cout << "child elements of robot at.." << i << " :"<< nameElem1  << endl;
            if(nameElem1 == "joint"){
                cout << "yippieeee" << endl;
                jointlist = elem.childNodes();
                for(int j=0; j<jointlist.size(); j++){
                    cout << "size of joint tag..: " << endl;
                    attrOrg = jointlist.at(j).toElement().attributeNode("name");
                    cout << attrJoint.value().toUtf8().constData() << endl;
                    nameElem2 = jointlist.at(j).toElement().tagName().toUtf8().constData();
                    if(nameElem2 == "origin"){
                        cout << "origin found..\n \n";
                        attrXYZ = jointlist.at(j).toElement().attributeNode("xyz");
                        attrRPY = jointlist.at(j).toElement().attributeNode("rpy");
                        cout << attrRPY.value().toUtf8().constData() << endl;
                        cout << attrXYZ.value().toUtf8().constData() << endl << endl;
                        ch = attrJoint.value().toUtf8().constData();
                        cout << "attrorg" << ch << endl;

                        if(ch == "gate_marker2_to_pool"){
                            ui->gate_marker2_to_pool->setText(attrXYZ.value());
                            ui->gate_marker2_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_buoy_blue_to_pool"){
                            ui->gate_buoy_blue_to_pool->setText(attrXYZ.value());
                            ui->gate_buoy_blue_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_buoy_green_to_pool"){
                             ui->gate_buoy_green_to_pool->setText(attrXYZ.value());
                             ui->gate_buoy_green_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_buoy_red_to_pool"){
                            ui->gate_buoy_red_to_pool->setText(attrXYZ.value());
                            ui->gate_buoy_red_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_marker3_to_pool"){
                            ui->gate_marker3_to_pool->setText(attrXYZ.value());
                            ui->gate_marker3_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_marker4_to_pool"){
                            ui->gate_marker4_to_pool->setText(attrXYZ.value());
                            ui->gate_marker4_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_marker5_to_pool"){
                            ui->gate_marker5_to_pool->setText(attrXYZ.value());
                            ui->gate_marker5_to_pool_r->setText(attrRPY.value());
                        }
                        else if (ch == "gate_shooter_to_pool"){
                            ui->gate_shooter_to_pool->setText(attrXYZ.value());
                            ui->gate_shooter_to_pool_r->setText(attrRPY.value());
                        }
                        else if(ch == "gate_dropper_to_pool"){
                            ui->gate_dropper_to_pool->setText(attrXYZ.value());
                            ui->gate_dropper_to_pool_r->setText(attrRPY.value());
                        }
                        else if(ch == "gate_L_rod_to_pool"){
                            ui->gate_L_rod_to_pool->setText(attrXYZ.value());
                            ui->gate_L_rod_to_pool_r->setText(attrRPY.value());
                        }


                    }
                }
            }else{
                cout << "why god why..!!" << endl;
            }
        } 
    }else
        cout << "root is NULL.. " << endl;
}




void gui_2::on_parser_clicked()
{
       if (!file->open(QIODevice::ReadOnly | QIODevice::Text)) {
        cout<<"file not opened...";
        return ;
       }
       if (!xmlDOC.setContent(file)) {
           printf("Not Again...!!!!");
           file->close();
           return ;
       }
       file->close();

        cout << "file parsed..!!" << endl;
        root = xmlDOC.documentElement();
        readFile();
}


void gui_2::on_change_clicked(){


    QFile *fileout = new QFile("/home/manvi/ros_ws/gui_2/write.urdf");
    if(!fileout->open(QIODevice::ReadWrite | QIODevice::Text)){
        cout << "file not opened...";
        return ;
    }
    else{
        cout << "file opened";
    }

        nodlist2 = xmlDOC.elementsByTagName("joint");
        cout << nodlist2.size()<< endl;

        for(int j=0; j<nodlist2.size(); j++){
            cout << "2nd loop.."<< endl;
            orglist = nodlist2.at(j).toElement().elementsByTagName("origin");
            joint = nodlist2.at(j).toElement().attributeNode("name");
            joint_name = joint.value().toUtf8().constData();
            cout << orglist.size() << endl;
            for(int i=0; i<orglist.size(); i++){
                xyz = orglist.at(i).toElement().attributeNode("xyz");
                rpy = orglist.at(i).toElement().attributeNode("rpy");
                cout << xyz.value().toUtf8().constData() << endl;

                if(joint_name== "gate_marker2_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_marker2_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_marker2_to_pool_r->text());
                }
                else if (joint_name == "gate_buoy_blue_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_buoy_blue_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_buoy_blue_to_pool_r->text());
                }
                else if (joint_name== "gate_buoy_green_to_pool"){
                     orglist.at(i).toElement().setAttribute("xyz",ui->gate_buoy_green_to_pool->text());
                     orglist.at(i).toElement().setAttribute("rpy",ui->gate_buoy_green_to_pool_r->text());
                }
                else if (joint_name== "gate_buoy_red_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_buoy_red_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_buoy_red_to_pool_r->text());
                }
                else if (joint_name== "gate_marker3_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_marker3_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_marker3_to_pool_r->text());
                }
                else if (joint_name== "gate_marker4_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_marker4_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_marker4_to_pool_r->text());
                }
                else if (joint_name== "gate_marker5_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_marker5_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_marker5_to_pool_r->text());
                }
                else if (joint_name== "gate_shooter_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_shooter_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_shooter_to_pool_r->text());
                }
                else if(joint_name== "gate_dropper_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_dropper_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_dropper_to_pool_r->text());
                }
                else if(joint_name== "gate_L_rod_to_pool"){
                    orglist.at(i).toElement().setAttribute("xyz",ui->gate_L_rod_to_pool->text());
                    orglist.at(i).toElement().setAttribute("rpy",ui->gate_L_rod_to_pool_r->text());

                }



          }
    }
        QTextStream stream (fileout);
        stream << xmlDOC.toString() << endl;
}



void gui_2::on_textEdit_textChanged()
{
    ui->textEdit->setText("yipiieee...!!!");
}

void gui_2::on_gate_marker2_to_pool_textChanged(const QString &arg1)
{

}
