package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Destacker_2 extends OpMode {

    public ColorSensor colour = null;

    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;
    public Servo Destacker = null;

    private int Toppos = 0;

    @Override
    public void loop() {
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        if(gamepad1.b && Base_Gripper.getPosition() == 0){
            Base_Gripper.setPosition(0.45); //open base gripper if it is closed
        }else if(gamepad1.b && Base_Gripper.getPosition() > 0){
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        if(gamepad1.a && Base_Pivot.getPosition() == 0){
            Base_Pivot.setPosition(0.5); //lift up base griper if it is down
        }else if(gamepad1.a && Base_Pivot.getPosition() > 0){
            Base_Pivot.setPosition(0); //lower base gripper if it is up
        }
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        if (gamepad1.x) {
            Destacker.setPosition(0);
            Base_Pivot.setPosition(0);
        }

        if (gamepad1.dpad_down) {
            Base_Pivot.setPosition(1);
        }

        if (gamepad1.y) {
            Destacker.setPosition(1);
            Base_Pivot.setPosition(0.25);
        }

        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        telemetry.addData("Destacker:", Destacker.getPosition());
        telemetry.addData("target pos:", Toppos);
        telemetry.update();
    }

    @Override
    public void init() {
        colour = hardwareMap.get(ColorSensor.class, "colour");

        Base_Gripper = hardwareMap.get(Servo.class,"Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class,"Base_Pivot");
        Destacker = hardwareMap.get(Servo.class,"Destacker");

        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Destacker.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        Destacker.setPosition(0);
    }
}
