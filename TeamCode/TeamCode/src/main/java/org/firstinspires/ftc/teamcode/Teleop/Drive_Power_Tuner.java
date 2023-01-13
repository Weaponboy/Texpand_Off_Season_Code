package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Drive_Power_Tuner extends OpMode {

    private int ticks = 0;
    static double ticksperdegree = 1.493;
    static double circumference = 30.144;

    private double vertical;
    private double horizontal;
    private double pivot;

    private int stakerpos_L = 0;
    private int stakerpos_LB = 0;
    private int stakerpos_R = 0;
    private int stakerpos_RB = 0;

    private double slow = 1;
    private double RF_slow = 1;
    private double RB_slow = 1;
    private double LF_slow = 1;
    private double LB_slow = 1;

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;


    @Override
    public void init() {
        //Driving motors config
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }

    @Override
    public void loop() {
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(slow*(-pivot + (vertical - RF_slow*horizontal)));
        RB.setPower(slow*(-pivot + (vertical + RB_slow*horizontal)));
        LF.setPower(slow*(pivot + (vertical + LF_slow*horizontal)));
        LB.setPower(slow*(pivot + (vertical - LB_slow*horizontal)));

        telemetry.addData("RF slow", RF_slow);
        telemetry.addData("RB slow", RB_slow);
        telemetry.addData("LF slow", LF_slow);
        telemetry.addData("LB slow", LB_slow);
        telemetry.addData("RF Power:", RF.getPower());
        telemetry.addData("RB Power:", RB.getPower());
        telemetry.addData("LF Power:", LF.getPower());
        telemetry.addData("LB Power:", LB.getPower());
        telemetry.update();

        if(gamepad1.dpad_down) {
            stakerpos_R = stakerpos_R + 1;

            if(stakerpos_R == 1){
                RF_slow = 1.4;
            } else if(stakerpos_R == 2) {
                RF_slow = 1.2;
            } else if(stakerpos_R == 3) {
                RF_slow = 1;
            } else if(stakerpos_R == 4) {
                RF_slow = 0.8;
            } else if(stakerpos_R == 5) {
                RF_slow = 0.6;
            }
            if(stakerpos_R > 5){
                stakerpos_R = 0;
            }

        }

        if(gamepad1.dpad_up) {
            stakerpos_RB = stakerpos_RB + 1;

            if(stakerpos_RB == 1){
                RB_slow = 1.4;
            } else if(stakerpos_RB == 2) {
                RB_slow = 1.2;
            } else if(stakerpos_RB == 3) {
                RB_slow = 1;
            } else if(stakerpos_RB == 4) {
                RB_slow = 0.8;
            } else if(stakerpos_RB == 5) {
                RB_slow = 0.6;
            }
            if(stakerpos_RB > 5){
                stakerpos_RB = 0;
            }

        }

        if(gamepad1.dpad_left) {
            stakerpos_L = stakerpos_L + 1;

            if(stakerpos_L == 1){
                LF_slow = 1.4;
            } else if(stakerpos_L == 2) {
                LF_slow = 1.2;
            } else if(stakerpos_L == 3) {
                LF_slow = 1;
            } else if(stakerpos_L == 4) {
                LF_slow = 0.8;
            } else if(stakerpos_L == 5) {
                LF_slow = 0.6;
            }
            if(stakerpos_L > 5){
                stakerpos_L = 0;
            }

        }

        if(gamepad1.dpad_right) {
            stakerpos_LB = stakerpos_LB + 1;

            if(stakerpos_LB == 1){
                LB_slow = 1.4;
            } else if(stakerpos_LB == 2) {
                LB_slow = 1.2;
            } else if(stakerpos_LB == 3) {
                LB_slow = 1;
            } else if(stakerpos_LB == 4) {
                LB_slow = 0.8;
            } else if(stakerpos_LB == 5) {
                LB_slow = 0.6;
            }
            if(stakerpos_LB > 5){
                stakerpos_LB = 0;
            }

        }
    }



    public void ResetEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
