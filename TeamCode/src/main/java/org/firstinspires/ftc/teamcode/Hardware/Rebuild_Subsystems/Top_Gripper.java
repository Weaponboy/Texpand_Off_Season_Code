package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Top_Gripper{

    HardwareMap hardwareMap;

    public Servo Top_Gripper;

    public DcMotorEx Top_Pivot;

    public Servo Top_Turn_Table;

    public PIDController pivot_controller;

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Top_Gripper = hardwareMap.get(Servo.class, "Top_Gripper");

        Top_Pivot = hardwareMap.get(DcMotorEx.class, "Top_Pivot");

        Top_Turn_Table = hardwareMap.get(Servo.class, "Top_Turn");

        Top_Turn_Table.setDirection(Servo.Direction.FORWARD);

        Top_Gripper.setDirection(Servo.Direction.FORWARD);

        Top_Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Top_Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
