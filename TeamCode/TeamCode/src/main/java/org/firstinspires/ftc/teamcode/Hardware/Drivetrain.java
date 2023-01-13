package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

public class Drivetrain {

    private int ticks = 0;
    static double ticksperdegree = 11.111;
    static double circumference = 30.144;

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();


    public Drivetrain() { }

    public int getTicks() {
        return ticks;
    }

    public void TurnDegrees(int degrees){
        ticks = (int) (ticksperdegree*degrees);
        ResetEncoders();

        RF.setTargetPosition(-ticks);
        LF.setTargetPosition(ticks);
        RB.setTargetPosition(-ticks);
        LB.setTargetPosition(ticks);

        RF.setPower(1);
        LF.setPower(1);
        RB.setPower(1);
        LB.setPower(1);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.isBusy()){
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void DriveDistance(double distance, double speed) {
        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (distance * ticksPerRevolution / wheelCircumference));


        // Set the target position for each motor
        RF.setTargetPosition(ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of each motor
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() < ticks - 20 || RB.getCurrentPosition() < ticks - 20 || LF.getCurrentPosition() < ticks - 20 || LB.getCurrentPosition() < ticks - 20) {

            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }
        }

        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
        try {
            Thread.sleep(10000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

    }

    public void StrafeDistance(double Strafe_cm, double power) {
        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (1.2*Strafe_cm * ticksPerRevolution / wheelCircumference));


        try {
            Thread.sleep(1000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        // Set the target position for each motor
        RF.setTargetPosition(-ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(-ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of each motor
        RF.setPower(power);
        RB.setPower(power);
        LF.setPower(power);
        LB.setPower(power);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() > (-ticks) + 20 || RB.getCurrentPosition() < ticks - 20 || LF.getCurrentPosition() < ticks - 20 || LB.getCurrentPosition() > (-ticks) + 20) {


            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }
        }

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
        try {
            Thread.sleep(10000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;
        //Driving motors config
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }

    public void ResetEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
