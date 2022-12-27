package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Top_gripper {

    public Servo Top_Gripper = null;
    public Servo Top_Pivot = null;

    private int Toppos = 0;
    private int TopgripperPos = 0;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();


    public Top_gripper() { }

    public void Top_Pivot(){
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(0.75);
            }else if(Toppos == 2){
                Top_Pivot.setPosition(1);
            }
            if(Toppos > 2){
                Toppos = 0;
            }

    }

    public void Top_Gripper(){
        TopgripperPos = TopgripperPos + 1;

        if(TopgripperPos == 1 && Top_Gripper.getPosition() == 0){
            Top_Gripper.setPosition(0.45); //lift up top griper if it is down
        }else if(TopgripperPos == 2 && Top_Gripper.getPosition() > 0){
            Top_Gripper.setPosition(0); //lower top gripper if it is up
        }

        if(TopgripperPos > 2){
            TopgripperPos = 0;
        }

    }

    public void Top_Pivot_single (){
        Top_Pivot.setPosition(0.3);
    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        Top_Gripper = hardwareMap.get(Servo.class,"Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class,"Top_Pivot");

        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.FORWARD);

        Top_Pivot.setPosition(1);
        Top_Gripper.setPosition(0.5);
    }
}
