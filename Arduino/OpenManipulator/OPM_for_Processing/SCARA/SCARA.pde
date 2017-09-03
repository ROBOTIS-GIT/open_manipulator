/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */

/**
 * this code is compatible with open_manipulator_scara.ino
**/

// Multiple Window
ChildApplet child;

// Control Interface
import controlP5.*;

// Init serial
import processing.serial.*;

// Shape variables
PShape link1, link2, link3, link4, link5;

// Model pose
float model_rot_x, model_rot_z, model_trans_x, model_trans_y, model_scale_factor;

// Serial variable
Serial opencr_port;

// Angle variable
float[] joint_angle   = new float[3];
float gripper_angle   = 0.0;

/*******************************************************************************
* Setting window size
*******************************************************************************/
void settings()
{
  size(900, 900, OPENGL);
}

/*******************************************************************************
* Setup
*******************************************************************************/
void setup()
{
  surface.setTitle("OpenManipulator SCARA");
  child = new ChildApplet();

  initShape();
  initView();

  connectOpenCR(0); // It is depend on laptop enviroments.
}

/*******************************************************************************
* Draw (loop function)
*******************************************************************************/
void draw()
{
  setWindow();

  drawTitle();
  drawWorldFrame();

  drawManipulator();
}

/*******************************************************************************
* Connect OpenCR
*******************************************************************************/
void connectOpenCR(int port_num)
{
  printArray(Serial.list());

  String port_name = Serial.list()[port_num];
  opencr_port = new Serial(this, port_name, 57600);
  opencr_port.bufferUntil('\n');
}

/*******************************************************************************
* Serial Interrupt
*******************************************************************************/
void serialEvent(Serial opencr_port)
{
  String opencr_string = opencr_port.readStringUntil('\n');
  opencr_string = trim(opencr_string);

  String[] cmd = split(opencr_string, ',');

  if (cmd[0].equals("angle"))
  {
    for (int cmd_cnt = 1; cmd_cnt < cmd.length; cmd_cnt++)
    {
      if (cmd_cnt == cmd.length-1)
      {
        gripper_angle = float(cmd[cmd_cnt]);
        println("gripper : " + cmd[cmd_cnt]);
      }
      else
      {
        joint_angle[cmd_cnt-1] = float(cmd[cmd_cnt]);
        print("joint " + cmd_cnt + ": " + cmd[cmd_cnt] + "  ");
      }
    }
  }
  else if (cmd[0].equals("end"))
  {
    println("end");
  }
  else
  {
    println("Error");
  }
}

/*******************************************************************************
* Init viewpoint and camera
*******************************************************************************/
void initView()
{
  float camera_y = height/2.0;
  float fov = 200/float(width) * PI/2;
  float camera_z = camera_y / tan(fov / 2.0);
  float aspect = float(width)/float(height);

  perspective(fov, aspect, camera_z/10.0, camera_z*10.0);

  // Eye position
  // Scene center
  // Upwards axis
  camera(width/2.0, height/2.0, height/2.0 * 4,
         width/2, height/2, 0,
         0, 1, 0);
}

/*******************************************************************************
* Get shape
*******************************************************************************/
void initShape()
{
  link1       = loadShape("meshes/scara/link1.obj");
  link2       = loadShape("meshes/scara/link2.obj");
  link3       = loadShape("meshes/scara/link3.obj");
  link4       = loadShape("meshes/scara/link4.obj");
  link5       = loadShape("meshes/scara/link5.obj");

  setJointAngle(0, 0, 0);
  gripperOn();
}

/*******************************************************************************
* Set window characteristic
*******************************************************************************/
void setWindow()
{
  lights();
  smooth();
  background(30);

  translate(width/2, height/2+200, 0);

  rotateX(radians(0));
  rotateZ(radians(-90));
}

/*******************************************************************************
* Draw title
*******************************************************************************/
void drawTitle()
{
  pushMatrix();
  rotateX(radians(0));
  rotateZ(radians(90));
  textSize(45);
  fill(255,204,102);
  text("OpenManipulator SCARA", -300,-460,0);
  textSize(20);
  fill(102,255,255);
  text("Press 'A','D','W','S'", -300,-420,0);
  text("And   'Q','E'",         -300,-390,0);
  popMatrix();
}

/*******************************************************************************
* Draw manipulator
*******************************************************************************/
void drawManipulator()
{
  scale(1.5 + model_scale_factor);

  pushMatrix();
  translate(-model_trans_x, -model_trans_y, 0);
  rotateX(model_rot_x);
  rotateY(radians(-10));
  rotateZ(model_rot_z);
  shape(link1);
  drawLocalFrame();

  translate(0, 0, 66.1);
  rotateZ(-joint_angle[0]);
  shape(link2);
  drawLocalFrame();

  translate(30, 0, 0);
  rotateZ(-joint_angle[1]);
  shape(link3);
  drawLocalFrame();

  translate(90.25, 0, 0);
  rotateZ(-joint_angle[2]);
  shape(link4);
  drawLocalFrame();

  translate(74, 0, 0);
  rotateX(-gripper_angle);
  shape(link5);
  drawLocalFrame();

  translate(30, 0, 0);

  drawLocalFrame();
  popMatrix();
}

/*******************************************************************************
* Draw world frame
*******************************************************************************/
void drawWorldFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 200, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, 200, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 200);
}

/*******************************************************************************
* Draw local frame
*******************************************************************************/
void drawLocalFrame()
{
  strokeWeight(10);
  stroke(255, 0, 0, 100);
  line(0, 0 ,0 , 100, 0, 0);

  strokeWeight(10);
  stroke(0, 255, 0, 100);
  line(0, 0, 0, 0, 100, 0);

  stroke(0, 0, 255, 100);
  strokeWeight(10);
  line(0, 0, 0, 0, 0, 100);
}

/*******************************************************************************
* Set joint angle
*******************************************************************************/
void setJointAngle(float angle1, float angle2, float angle3)
{
  joint_angle[0] = angle1;
  joint_angle[1] = angle2;
  joint_angle[2] = angle3;
}

/*******************************************************************************
* Gripper on
*******************************************************************************/
void gripperOn()
{
  gripper_angle = 0.0;
}

/*******************************************************************************
* Gripper off
*******************************************************************************/
void gripperOff()
{
  gripper_angle = 1.0;
}

/*******************************************************************************
* Mouse drag event
*******************************************************************************/
void mouseDragged()
{
  model_rot_z -= (mouseX - pmouseX) * 0.01;
  model_rot_x -= (mouseY - pmouseY) * 0.01;
}

/*******************************************************************************
* Key press event
*******************************************************************************/
void keyPressed()
{
  if      (key == 'a') model_trans_x      -= 50;
  else if (key == 'd') model_trans_x      += 50;
  else if (key == 's') model_trans_y      += 50;
  else if (key == 'w') model_trans_y      -= 50;
  else if (key == 'q') model_scale_factor += 0.5;
  else if (key == 'e') model_scale_factor -= 0.5;
  else if (key == 'i') model_trans_x = model_trans_y = model_scale_factor = model_rot_z = model_rot_x = 0;
}

/*******************************************************************************
* Controller Window
*******************************************************************************/
class ChildApplet extends PApplet
{
  ControlP5 cp5;

  Textlabel headLabel;
  Knob joint1, joint2, joint3, gripper;
  Slider2D slider2d;

  float[] set_joint_angle = new float[3];
  float set_gripper_pos;

  boolean onoff_flag = false;

  public ChildApplet()
  {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings()
  {
    size(400, 600);
    smooth();
  }
  public void setup()
  {
    surface.setTitle("Control Interface");
    cp5 = new ControlP5(this);

/*******************************************************************************
* Init Tab
*******************************************************************************/
    cp5.addTab("Task Space Control")
       .setColorBackground(color(242,56,39))
       .setColorLabel(color(255))
       .setColorActive(color(242,211,39))
       ;

    cp5.addTab("Motion")
       .setColorBackground(color(242,56,39))
       .setColorLabel(color(255))
       .setColorActive(color(242,211,39))
       ; 

    cp5.getTab("default")
       .activateEvent(true)
       .setLabel("Joint Space Control")
       .setId(1)
       ;

    cp5.getTab("Task Space Control")
       .activateEvent(true)
       .setId(2)
       ;

    cp5.getTab("Motion")
       .activateEvent(true)
       .setId(3)
       ;

/*******************************************************************************
* Init Joint Space Controller
*******************************************************************************/
    headLabel = cp5.addTextlabel("Label")
                   .setText("Controller for OpenManipulator SCARA")
                   .setPosition(4,17)
                   .setColorValue(0xffffff00)
                   .setFont(createFont("arial",20))
                   ;

    cp5.addToggle("Controller_OnOff")
       .setPosition(0,50)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

    joint1 = cp5.addKnob("joint1")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(70,130)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint2 = cp5.addKnob("joint2")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(220,130)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    joint3 = cp5.addKnob("joint3")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(70,250)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    gripper = cp5.addKnob("gripper")
             .setRange(-3.14,3.14)
             .setValue(0)
             .setPosition(220,250)
             .setRadius(50)
             .setDragDirection(Knob.HORIZONTAL)
             .setFont(createFont("arial",10))
             .setColorForeground(color(255))
             .setColorBackground(color(0, 160, 100))
             .setColorActive(color(255,255,0))
             ;

    cp5.addButton("Origin")
       .setValue(0)
       .setPosition(0,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Basic")
       .setValue(0)
       .setPosition(320,350)
       .setSize(80,40)
       .setFont(createFont("arial",13))
       .setColorForeground(color(150,150,0))
       .setColorBackground(color(100, 160, 0))
       ;

    cp5.addButton("Send_Joint_Angle")
       .setValue(0)
       .setPosition(0,400)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Set_Gripper")
       .setValue(0)
       .setPosition(0,460)
       .setSize(400,40)
       .setFont(createFont("arial",15))
       ;

    cp5.addToggle("Gripper_OnOff")
       .setPosition(0,520)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    slider2d = cp5.addSlider2D("Drawing")
                  .setPosition(70,240)
                  .setSize(260,150)
                  .setMinMax(130,250,-130,100)
                  .setValue(0,240)
                  ;

    cp5.addToggle("Drawing_Gripper_Set")
       .setPosition(0,520)
       .setSize(400,40)
       .setMode(Toggle.SWITCH)
       .setFont(createFont("arial",15))
       .setColorActive(color(196, 196, 196))
       .setColorBackground(color(255, 255, 153))
       ;

/*******************************************************************************
* Init Task Space Controller
*******************************************************************************/
    cp5.addButton("Motion_Start")
       .setValue(0)
       .setPosition(0,200)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

    cp5.addButton("Motion_Stop")
       .setValue(0)
       .setPosition(0,330)
       .setSize(400,100)
       .setFont(createFont("arial",15))
       ;

/*******************************************************************************
* Set Tap UI
*******************************************************************************/
    cp5.getController("Label").moveTo("global");
    cp5.getController("Controller_OnOff").moveTo("global");

    cp5.getController("Drawing").moveTo("Task Space Control");
    cp5.getController("Drawing_Gripper_Set").moveTo("Task Space Control");

    cp5.getController("Motion_Start").moveTo("Motion");
    cp5.getController("Motion_Stop").moveTo("Motion");
  }

  public void draw()
  {
    background(1,35,64);
  }

/*******************************************************************************
* Init Function of Joint Space Controller
*******************************************************************************/
  void Controller_OnOff(boolean flag)
  {
    onoff_flag = flag;
    if (onoff_flag)
    {
      joint1.setValue(joint_angle[0]);
      joint2.setValue(joint_angle[1]);
      gripper.setValue(gripper_angle);

      opencr_port.write("mnp"   + ',' +
                        "ready" + '\n');
      println("OpenManipulator SCARA Ready!!!");
    }
    else
    {
      opencr_port.write("mnp"  + ',' +
                        "end"  + '\n');
      println("OpenManipulator SCARA End...");
    }
  }

  void joint1(float angle)
  {
    joint_angle[0] = angle;
  }

  void joint2(float angle)
  {
    joint_angle[1] = angle;
  }

  void joint3(float angle)
  {
    joint_angle[2] = angle;
  }

  void gripper(float angle)
  {
    gripper_angle = angle;
  }

  public void Origin(int theValue)
  {
    if (onoff_flag)
    {
      joint_angle[0] = 0.0;
      joint_angle[1] = 0.0;
      joint_angle[2] = 0.0;

      joint1.setValue(joint_angle[0]);
      joint2.setValue(joint_angle[1]);
      joint3.setValue(joint_angle[2]);
      gripper.setValue(gripper_angle);

      opencr_port.write("joint"        + ',' +
                        joint_angle[0] + ',' +
                        joint_angle[1] + ',' +
                        joint_angle[2] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Basic(int theValue)
  {
    if (onoff_flag)
    {
      set_joint_angle[0] = -60.0 * PI/180.0;
      set_joint_angle[1] = 20.0 * PI/180.0;
      set_joint_angle[2] = 40.0 * PI/180.0;

      opencr_port.write("joint"            + ',' +
                        set_joint_angle[0] + ',' +
                        set_joint_angle[1] + ',' +
                        set_joint_angle[2] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Send_Joint_Angle(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("joint"        + ',' +
                        joint_angle[0] + ',' +
                        joint_angle[1] + ',' +
                        joint_angle[2] + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Set_Gripper(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("gripper"  + ',' +
                        gripper_angle + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  void Gripper_OnOff(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        gripper.setValue(0.0);
        opencr_port.write("grip"  + ',' +
                          "off" + '\n');
      }
      else
      {
        gripper.setValue(-0.5);
        opencr_port.write("grip"  + ',' +
                          "on" + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Task Space Controller
*******************************************************************************/
  void Drawing()
  {
    float posX, posY;

    posX = slider2d.getArrayValue()[0] * 0.001;
    posY = slider2d.getArrayValue()[1] * 0.001;

    opencr_port.write("pos"     + ',' +
                      posY      + ',' +
                      posX      + '\n');

    println("x = " + posY + " y = " + posX);
  }

  void Drawing_Gripper_Set(boolean flag)
  {
    if (onoff_flag)
    {
      if (flag)
      {
        gripper.setValue(0.0);
        opencr_port.write("grip"  + ',' +
                          "off" + '\n');
      }
      else
      {
        gripper.setValue(-0.5);
        opencr_port.write("grip"  + ',' +
                          "on" + '\n');
      }
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

/*******************************************************************************
* Init Function of Motion
*******************************************************************************/
  public void Motion_Start(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }

  public void Motion_Stop(int theValue)
  {
    if (onoff_flag)
    {
      opencr_port.write("motion"  + ',' +
                        "stop"       + '\n');
    }
    else
    {
      println("Please, Set On Controller");
    }
  }
}