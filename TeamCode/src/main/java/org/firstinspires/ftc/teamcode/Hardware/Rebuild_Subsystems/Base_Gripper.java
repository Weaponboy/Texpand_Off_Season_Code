package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Base_Gripper {

    HardwareMap hardwareMap;

    public Servo Base_Gripper;

    public Servo Base_Pivot;

    public Servo Destacker_Left = null;
    public Servo Destacker_Right = null;

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Base_Gripper = hardwareMap.get(Servo.class, "Base_Gripper");

        Base_Pivot = hardwareMap.get(Servo.class, "Base_Pivot");

        Destacker_Left = hardwareMap.get(Servo.class,"Destacker Left");

        Destacker_Right = hardwareMap.get(Servo.class,"Destacker Right");

        Destacker_Left.setDirection(Servo.Direction.REVERSE);

    }
    
    public void Destacker_Position(double Position){
        Destacker_Left.setPosition(Position);
        Destacker_Right.setPosition(Position);
    }
}
