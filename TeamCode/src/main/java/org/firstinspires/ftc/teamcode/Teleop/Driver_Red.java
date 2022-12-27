package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.bottom_gripper;

public class Driver_Red extends OpMode {


    Drivetrain drive = new Drivetrain();
    bottom_gripper B_grip = new bottom_gripper();
    Slides slide = new Slides();
    Top_gripper top = new Top_gripper();

    private double vertical;
    private double horizontal;
    private double pivot;

    private int Toppos = 0;
    private int stakerpos = 0;

    private boolean conefound = false;
    private boolean extending = false;
    private double slow = 1;
    private boolean lowering = false;

    public ColorSensor colour = null;

    @Override
    public void init() {
        drive.init(hardwareMap);
        B_grip.init(hardwareMap);
        slide.init(hardwareMap);
        top.init(hardwareMap);

        colour = hardwareMap.get(ColorSensor.class, "colour");
    }

    @Override
    public void loop() {
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower(slow*(-pivot + (vertical - horizontal)));
        drive.RB.setPower(slow*(-pivot + (vertical + horizontal)));
        drive.LF.setPower(slow*(pivot + (vertical + horizontal)));
        drive.LB.setPower(slow*(pivot + (vertical - horizontal)));

        if(gamepad1.dpad_up){
            B_grip.destacker1();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.b){
            B_grip.Base_Gripper();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.b){
            B_grip.Base_Gripper();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.a){
            B_grip.Base_Pivot();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.dpad_down){
            top.Top_Pivot_single();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.y){
            top.Top_Gripper();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.x){
            top.Top_Pivot();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.dpad_right && slow == 1){
            slow = 0.4;
        }else if(gamepad1.dpad_right && slow < 1){
            slow = 1;
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.left_bumper){
            slide.Up();
        }else if(gamepad1.right_bumper){
            slide.Down();
        }else{
            slide.Off();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if (gamepad1.dpad_left){
            slide.Top_pole();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if (gamepad1.left_trigger > 0){
            collect();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.right_trigger > 0){
            slide_down();
        }

        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()){
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;
        }else if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        telemetry.addData("Destacker:", B_grip.Destacker.getPosition());
        telemetry.addData("Red:", colour.red());
        telemetry.addData("Green:", colour.green());
        telemetry.addData("Blue:", colour.blue());
        telemetry.addData("motor ticks:", slide.Extend.getCurrentPosition());
        telemetry.addData("motor ticks Right:", slide.Right_Slide.getCurrentPosition());
        telemetry.addData("motor ticks Left:", slide.Left_Slide.getCurrentPosition());
        telemetry.update();
    }


    public void collect (){
        slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && slide.Extend.getCurrentPosition() > -1900){
            if(slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()){
                slide.Right_Slide.setPower(0);
                slide.Left_Slide.setPower(0);

                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                slide.Right_Slide.setPower(-0.9);
                slide.Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            slide.Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", slide.Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        slide.Extend.setPower(0);
        if(conefound) {
            if (slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()) {
                slide.Right_Slide.setPower(0);
                slide.Left_Slide.setPower(0);

                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            } else if (lowering) {
                slide.Right_Slide.setPower(-0.9);
                slide.Left_Slide.setPower(-0.9);
            }
            top.Top_Pivot.setPosition(1);
            top.Top_Gripper.setPosition(0.5);
            B_grip.Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            if (B_grip.Base_Gripper.getPosition() == 0) {
                slide.Extend.setTargetPosition(20);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                B_grip.Base_Pivot.setPosition(1);
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                while (slide.Extend.isBusy()) {
                    if (slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()) {
                        slide.Right_Slide.setPower(0);
                        slide.Left_Slide.setPower(0);

                        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    } else if (lowering) {
                        slide.Right_Slide.setPower(-0.9);
                        slide.Left_Slide.setPower(-0.9);
                    }
                    B_grip.Base_Gripper.setPosition(0);
                    B_grip.Destacker.setPosition(0.2);

                    slide.Extend.setPower(1);
                }
                if (slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()) {
                    slide.Right_Slide.setPower(0);
                    slide.Left_Slide.setPower(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                } else if (lowering) {
                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);
                }
                slide.Extend.setPower(0);
                if (B_grip.Base_Pivot.getPosition() == 1) {
                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
                    top.Top_Gripper.setPosition(0);
                    if (top.Top_Gripper.getPosition() == 0) {
                        B_grip.Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(100);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }
                        if (B_grip.Base_Gripper.getPosition() <= 0.45) {
                            top.Top_Pivot.setPosition(0.4);
                            try {
                                Thread.sleep(50);
                            } catch (Exception e) {
                                System.out.println(e.getMessage());
                            }
                            B_grip.Base_Pivot.setPosition(0.35);
                        }

                    }
                }
            }
            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            slide.Extend.setTargetPosition(-50);
            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slide.Extend.isBusy()) {
                if(slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()){
                    slide.Right_Slide.setPower(0);
                    slide.Left_Slide.setPower(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);
                }
                slide.Extend.setPower(0.8);
            }
            slide.Extend.setPower(0);
        }
    }//End of collect method

    public void slide_down(){
        top.Top_Gripper.setPosition(0.45);
        if(top.Top_Gripper.getPosition() == 0.45) {
            slide.Right_Slide.setTargetPosition(0);
            slide.Left_Slide.setTargetPosition(0);
            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            top.Top_Pivot.setPosition(1);
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
            lowering = true;
        }
    }

}
