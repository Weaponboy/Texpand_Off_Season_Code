package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
@Disabled
@TeleOp
public class Driver_Blue extends OpMode {

    double De_Pos_1 = 0.0;
    double De_Pos_2 = 0.15;
    double De_Pos_3 = 0.55;
    double De_Pos_4 = 0.7;
    double De_Pos_5 = 0.8;

    private double vertical;
    private double horizontal;
    private double pivot;

    private double Base_Pivot_Collect = 0.08;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;

    private double Top_Gripper_Collect = 0.33;

    private double Top_Gripper_Collect_Wide = 0.36;

    private int Toppos = 0;

    private int stakerpos = 0;

    private double Destack_position = 0;

    private boolean conefound = false;

    private double slow = 0.4;

    private boolean lowering = false;

    Drivetrain drive = new Drivetrain();

    Slides slide = new Slides();

    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();

    Top_gripper top = new Top_gripper();

    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void loop() {
        runtime.reset();

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower(slow*(-pivot + (vertical - horizontal)));
        drive.RB.setPower(1.18*(slow*(-pivot + (vertical + horizontal))));
        drive.LF.setPower(slow*(pivot + (vertical + horizontal)));
        drive.LB.setPower(1.18*(slow*(pivot + (vertical - horizontal))));

        //destack
        if (gamepad2.dpad_up) {
            DestackPosition();
        }

        //Toggle base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
            B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        if (gamepad1.dpad_up) {
            bottom.Destacker_Left.setPosition(De_Pos_1);
            bottom.Destacker_Right.setPosition(De_Pos_1);
            bottom.Base_Pivot.setPosition(0.12);
        }
        if (gamepad1.right_bumper) {
            bottom.Destacker_Left.setPosition(De_Pos_2);
            bottom.Destacker_Right.setPosition(De_Pos_2);
        }
        if (gamepad1.y) {
            bottom.Destacker_Left.setPosition(De_Pos_3);
            bottom.Destacker_Right.setPosition(De_Pos_3);
        }
        if (gamepad1.b) {
            bottom.Destacker_Left.setPosition(De_Pos_4);
            bottom.Destacker_Right.setPosition(De_Pos_4);
        }
        if (gamepad1.a) {
            bottom.Destacker_Left.setPosition(De_Pos_5);
            bottom.Destacker_Right.setPosition(De_Pos_5);
        }


        //Stop slides if finished running
        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.back || gamepad2.back){
            CollectIntoBot();
        }

        //Toggle Base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
            B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        //Reduce robot speed
        if(gamepad1.start && slow == 0.6){
            slow = 0.4;
        }else if(gamepad1.start && slow < 0.6){
            slow = 0.6;
        }

        //Sleep
        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        //Stop slides if finished running
        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //toggle base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
           B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        //toggle possition of top pivot
        if (gamepad2.x || gamepad1.x) {
           ToggleTopPivot();
        }

        //set top pivot position to drop position
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            top.Top_Pivot.setPosition(0);
        }

        //toggle positioin of top gripper
        if(gamepad2.y && top.Top_Gripper.getPosition() == 0){
            TopGripperOpen();
        }else if(gamepad2.y && top.Top_Gripper.getPosition() > 0){
            TopGripperClosed();
        }

        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper){
            FullCycle();
        }

        //set slides to High pole
        if(gamepad1.dpad_left || gamepad2.dpad_left){
           DropOffAtHigh();
        }

        if(gamepad2.a && bottom.Base_Pivot.getPosition() != 0.85){
           B_Pivot_Up();
        }else if(gamepad2.a && bottom.Base_Pivot.getPosition() != Base_Pivot_Collect ){
           B_Pivot_Down();
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right || gamepad2.dpad_right){
           DropOffAtMedium();
        }

        //Stop slides if finished running
        if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //extend slides to collect cone
        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
            CollectAndTopPivotOver();
        }

//bring slides back to bottom
        if(gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0){
            SlidesToZero();
        }

//stop slides if finished
        if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;
        }else if(lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

    }

    //init
    @Override
    public void init() {

        drive.init(hardwareMap);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        slide.init(hardwareMap);

        slide.customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

    }
    public void DestackPosition(){
        stakerpos = stakerpos + 1;

        if(stakerpos == 1){
            Destack_position = De_Pos_1;
            bottom.Base_Pivot.setPosition(0.12);
        } else if(stakerpos == 2) {
            Destack_position = De_Pos_2;
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        } else if(stakerpos == 3) {
            Destack_position = De_Pos_3;
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        } else if(stakerpos == 4) {
            Destack_position = De_Pos_4;
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        } else if(stakerpos == 5) {
            Destack_position = De_Pos_5;
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if(stakerpos > 5){
            stakerpos = 0;
        }
        bottom.Destacker_Left.setPosition(Destack_position);
        bottom.Destacker_Right.setPosition(Destack_position);
    }
    public void B_Grip_Open(){
        try {
            Thread.sleep(50);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Gripper.setPosition(0.4); //close base gripper if it is open
    }
    public void B_Grip_Closed(){
        try {
            Thread.sleep(50);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Gripper.setPosition(0); //close base gripper if it is open
    }
    public void B_Pivot_Down(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
    }
    public void B_Pivot_Up(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Pivot.setPosition(0.85);
    }
    public void CollectIntoBot(){

        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Extend.setPower(-1);

        conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 50;


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
            conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 65;
            slide.Extend.setPower(-1);
            try {
                Thread.sleep(20);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:",slide.sensorRange.getDistance(DistanceUnit.MM));
            telemetry.addData("motor ticks:", slide.Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        slide.Extend.setPower(0);

        if(conefound) {
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            bottom.Base_Gripper.setPosition(0);

            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                slide.Right_Slide.setPower(0);
                slide.Left_Slide.setPower(0);

                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                slide.Right_Slide.setPower(-0.9);
                slide.Left_Slide.setPower(-0.9);
            }

            top.Top_Pivot.setPosition(0.6);

            top.Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

            bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

            if (bottom.Destacker_Left.getPosition() < 0.6){

                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

            }

            try {
                Thread.sleep(125);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            if(bottom.Base_Gripper.getPosition() == 0){

                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (slide.Extend.isBusy()) {
                    if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                        slide.Right_Slide.setPower(0);
                        slide.Left_Slide.setPower(0);

                        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        slide.Right_Slide.setPower(-0.9);
                        slide.Left_Slide.setPower(-0.9);
                    }

                    bottom.Base_Gripper.setPosition(0);

                    bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

                    slide.Extend.setPower(1);

                }

                if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                    slide.Right_Slide.setPower(0);
                    slide.Left_Slide.setPower(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);
                }

                slide.Extend.setPower(0);

                bottom.Destacker_Left.setPosition(0.8);
                bottom.Destacker_Right.setPosition(0.8);

                if(bottom.Base_Pivot.getPosition() > 0.7) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    top.Top_Pivot.setPosition(1);

                    bottom.Base_Gripper.setPosition(0.4);

                    bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                    try {
                        Thread.sleep(250);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    top.Top_Gripper.setPosition(0);
                }

            }

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            slide.Extend.setTargetPosition(0);
            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slide.Extend.isBusy()) {
                if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
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
    }
    public void ToggleTopPivot(){
        Toppos = Toppos + 1;

        if(Toppos == 1){
            top.Top_Pivot.setPosition(0.22);
        }else if(Toppos == 2){
            top.Top_Pivot.setPosition(1);
        }else if(Toppos == 3){
            top.Top_Pivot.setPosition(0.6);
        }
        if(Toppos > 3){
            Toppos = 0;
        }
    }
    public void TopGripperOpen(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        top.Top_Gripper.setPosition(Top_Gripper_Collect);
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);
    }
    public void TopGripperClosed(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        top.Top_Gripper.setPosition(0);
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);
    }
    public void FullCycle(){
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Extend.setPower(-1);
            conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 50;

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
                conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 65;

                slide.Extend.setPower(-1);

                telemetry.addData("Blue:", slide.sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", slide.Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            slide.Extend.setPower(0);

            if(conefound) {

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                bottom.Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                    slide.Right_Slide.setPower(0);
                    slide.Left_Slide.setPower(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);
                }

                top.Top_Pivot.setPosition(0.77);
                top.Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                if(bottom.Base_Gripper.getPosition() == 0){

                    slide.Extend.setTargetPosition(0);
                    slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (slide.Extend.isBusy()) {
                        if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                            slide.Right_Slide.setPower(0);
                            slide.Left_Slide.setPower(0);

                            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            slide.Right_Slide.setPower(-0.9);
                            slide.Left_Slide.setPower(-0.9);
                        }
                        bottom.Base_Gripper.setPosition(0);

                        bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

                        top.Top_Pivot.setPosition(0.6);

                        slide.Extend.setPower(1);
                    }

                    if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                        slide.Right_Slide.setPower(0);
                        slide.Left_Slide.setPower(0);

                        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        slide.Right_Slide.setPower(-0.9);
                        slide.Left_Slide.setPower(-0.9);
                    }
                    slide.Extend.setPower(0);

                    bottom.Destacker_Left.setPosition(0.8);
                    bottom.Destacker_Right.setPosition(0.8);

                    if(bottom.Base_Pivot.getPosition() > 0.6) {

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        bottom.Base_Gripper.setPosition(0.4);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Pivot.setPosition(1);

                        try {
                            Thread.sleep(250);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(150);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Pivot.setPosition(0.4);

                        try {
                            Thread.sleep(75);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
                    }

                }

                slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (slide.Extend.isBusy()) {
                    if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
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
                bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
            }

            try {
                Thread.sleep(200);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            slide.Right_Slide.setTargetPosition(1900);
            slide.Left_Slide.setTargetPosition(1900);
            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){
                slide.Right_Slide.setPower(1);
                slide.Left_Slide.setPower(1);
            }
            slide.Right_Slide.setPower(0.005);
            slide.Left_Slide.setPower(0.005);

            top.Top_Pivot.setPosition(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            try {
                Thread.sleep(400);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            top.Top_Gripper.setPosition(Top_Gripper_Collect);
            if(top.Top_Gripper.getPosition() == Top_Gripper_Collect) {

                slide.Right_Slide.setTargetPosition(0);
                slide.Left_Slide.setTargetPosition(0);

                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                top.Top_Pivot.setPosition(0.6);

                slide.Right_Slide.setPower(-0.9);
                slide.Left_Slide.setPower(-0.9);

                lowering = true;
            }
    }
    public void CollectAndTopPivotOver(){
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Extend.setPower(-1);
            conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 65;

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
                conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

                slide.Extend.setPower(-1);

                try {
                    Thread.sleep(10);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                telemetry.addData("Blue:", slide.sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", slide.Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            slide.Extend.setPower(0);

            if(conefound) {

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                bottom.Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                    slide.Right_Slide.setPower(0);
                    slide.Left_Slide.setPower(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);
                }

                top.Top_Pivot.setPosition(0.77);
                top.Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                if(bottom.Base_Gripper.getPosition() == 0){

                    slide.Extend.setTargetPosition(0);
                    slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (slide.Extend.isBusy()) {
                        if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                            slide.Right_Slide.setPower(0);
                            slide.Left_Slide.setPower(0);

                            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            slide.Right_Slide.setPower(-0.9);
                            slide.Left_Slide.setPower(-0.9);
                        }
                        bottom.Base_Gripper.setPosition(0);

                        bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

                        top.Top_Pivot.setPosition(0.6);

                        slide.Extend.setPower(1);
                    }

                    if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
                        slide.Right_Slide.setPower(0);
                        slide.Left_Slide.setPower(0);

                        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        slide.Right_Slide.setPower(-0.9);
                        slide.Left_Slide.setPower(-0.9);
                    }
                    slide.Extend.setPower(0);

                    bottom.Destacker_Left.setPosition(0.8);
                    bottom.Destacker_Right.setPosition(0.8);

                    if(bottom.Base_Pivot.getPosition() > 0.6) {

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        bottom.Base_Gripper.setPosition(0.4);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Pivot.setPosition(1);

                        try {
                            Thread.sleep(150);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(150);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        top.Top_Pivot.setPosition(0.4);

                        try {
                            Thread.sleep(75);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
                    }

                }

                slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (slide.Extend.isBusy()) {
                    if(slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()){
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
                bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
            }

    }
    public void DropOffAtHigh(){
        slide.Right_Slide.setTargetPosition(1900);
        slide.Left_Slide.setTargetPosition(1900);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);
        }
        slide.Right_Slide.setPower(0.005);
        slide.Left_Slide.setPower(0.005);

        top.Top_Pivot.setPosition(0);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DropOffAtMedium(){
        slide.Right_Slide.setTargetPosition(900);
        slide.Left_Slide.setTargetPosition(900);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);
        }
        slide.Right_Slide.setPower(0.005);
        slide.Left_Slide.setPower(0.005);

        top.Top_Pivot.setPosition(0);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void SlidesToZero(){
        top.Top_Gripper.setPosition(Top_Gripper_Collect);
        if(top.Top_Gripper.getPosition() == Top_Gripper_Collect) {

            slide.Right_Slide.setTargetPosition(0);
            slide.Left_Slide.setTargetPosition(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            top.Top_Pivot.setPosition(0.6);

            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);

            lowering = true;
        }
    }
}

