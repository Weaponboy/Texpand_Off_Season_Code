package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
@Disabled
@TeleOp
public class Driver_Blue extends OpMode {

    private boolean conefound = false;
    private double slow = 1;
    private boolean lowering = false;

    Drivetrain drive = new Drivetrain();
    Bottom_Gripper_Assembly B_grip = new Bottom_Gripper_Assembly();
    Slides slide = new Slides();
    Top_gripper top = new Top_gripper();

    private double vertical;
    private double horizontal;
    private double pivot;

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
//
//        vertical = -gamepad1.right_stick_y;
//        horizontal = gamepad1.right_stick_x;
//        pivot = gamepad1.left_stick_x;
//
//        drive.RF.setPower(slow*(-pivot + (vertical - horizontal)));
//        drive.RB.setPower(slow*(-pivot + (vertical + horizontal)));
//        drive.LF.setPower(slow*(pivot + (vertical + horizontal)));
//        drive.LB.setPower(slow*(pivot + (vertical - horizontal)));
//
//        //done
//        if(gamepad1.dpad_up){
//            B_grip.destacker1();
//        }
//
//        //done
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.options){
//            slide.Bring_slides_down();
//        }
//
//        //done
//        if(gamepad1.b){
//            B_grip.Base_Gripper();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.b){
//            B_grip.Base_Gripper();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.a){
//            B_grip.Base_Pivot();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.dpad_down){
//            top.Top_Pivot_single();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.y){
//            top.Top_Gripper();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.x){
//            top.Top_Pivot();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.start && slow == 1){
//            slow = 0.4;
//        }else if(gamepad1.start && slow < 1){
//            slow = 1;
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if(gamepad1.left_bumper){
//            slide.Up();
//        }else if(gamepad1.right_bumper){
//            slide.Down();
//        }else if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
//            slide.Off();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if (gamepad1.dpad_left){
//            slide.Top_pole();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        //done
//        if (gamepad1.left_trigger > 0){
//            slide.Full_Cycle_To_High_Pole();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        if(gamepad1.right_trigger > 0){
//            slide.slide_down();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        if(gamepad1.dpad_right){
//            slide.Medium_pole();
//        }
//
//        if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
//        if(slide.Right_Slide.getCurrentPosition() < 50 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 50 && !slide.Left_Slide.isBusy()){
//            slide.Right_Slide.setPower(0);
//            slide.Left_Slide.setPower(0);
//
//            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            lowering = false;
//        }else if(lowering){
//            slide.Right_Slide.setPower(-0.9);
//            slide.Left_Slide.setPower(-0.9);
//        }
//
    }




}

