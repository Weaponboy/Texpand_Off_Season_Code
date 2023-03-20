package org.firstinspires.ftc.teamcode.Hardware.Sub_Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Top_gripper {

    private int Toppos = 0;
    private int stakerpos = 0;
    private int TopgripperPos = 0;

    public Servo Top_Gripper = null;
    public Servo Top_Pivot = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public Top_gripper() {}

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        Top_Gripper = hardwareMap.get(Servo.class,"Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class,"Top_Pivot");

        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.REVERSE);

        Top_Gripper.setPosition(0);
    }

}
