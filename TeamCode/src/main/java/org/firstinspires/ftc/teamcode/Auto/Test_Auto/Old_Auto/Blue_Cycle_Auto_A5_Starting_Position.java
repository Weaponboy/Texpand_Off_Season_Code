package org.firstinspires.ftc.teamcode.Auto.Test_Auto.Old_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;

@Autonomous
@Disabled
public class Blue_Cycle_Auto_A5_Starting_Position extends LinearOpMode {


    private double vertical;
    private double horizontal;
    private double pivot;

    private int Toppos = 0;
    private int stakerpos = 0;
    private double basepos = 0;

    private boolean conefound = false;
    private boolean extending = false;
    private double slow = 1;
    private boolean lowering = false;
    public double Destack_position = 0;


    //Drivetrain object
    Drivetrain drive = new Drivetrain();
    Slides slide = new Slides();
    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();
    Top_gripper top = new Top_gripper();




    @Override
    public void runOpMode() throws InterruptedException {

//        colour = hardwareMap.get(ColorSensor.class, "colour");
//
//        RF = hardwareMap.get(DcMotor.class, "RF");
//        LF = hardwareMap.get(DcMotor.class, "LF");
//        RB = hardwareMap.get(DcMotor.class, "RB");
//        LB = hardwareMap.get(DcMotor.class, "LB");
//
//        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
//        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");
//
//        Extend = hardwareMap.get(DcMotor.class, "Extend");
//
//        Base_Gripper = hardwareMap.get(Servo.class, "Base_Gripper");
//        Base_Pivot = hardwareMap.get(Servo.class, "Base_Pivot");
//        Top_Gripper = hardwareMap.get(Servo.class, "Top_Gripper");
//        Top_Pivot = hardwareMap.get(Servo.class, "Top_Pivot");
//        Destacker_Left = hardwareMap.get(Servo.class, "Destacker Left");
//        Destacker_Right = hardwareMap.get(Servo.class, "Destacker Right");
//
//        Destacker_Left.setDirection(Servo.Direction.REVERSE);
//        Base_Gripper.setDirection(Servo.Direction.FORWARD);
//        Base_Pivot.setDirection(Servo.Direction.FORWARD);
//        Top_Gripper.setDirection(Servo.Direction.FORWARD);
//        Top_Pivot.setDirection(Servo.Direction.REVERSE);
//
//        RF.setDirection(DcMotorSimple.Direction.FORWARD);
//        LF.setDirection(DcMotorSimple.Direction.REVERSE);
//        RB.setDirection(DcMotorSimple.Direction.FORWARD);
//        LB.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        RF.setPower(0);
//        LF.setPower(0);
//        RB.setPower(0);
//        LB.setPower(0);
        drive.init(hardwareMap, 1);
        slide.init(hardwareMap, 1);
        top.init(hardwareMap);
        bottom.init(hardwareMap);

        top.Top_Gripper.setPosition(0);
        bottom.Base_Gripper.setPosition(0.4);
        bottom.Base_Pivot.setPosition(0);
        top.Top_Pivot.setPosition(0.4);

        waitForStart();

        drive.DriveDistanceLong(140, 0.6);

        drive.TurnDegreesLeft(90);

        drive.StrafeDistance(25, 0.6);

        drive.DriveDistance(24, 0.6);

        drive.TurnDegreesLeft(13);

        drive.StrafeDistance_Left(15, 0.6);

        drive.DriveDistance(5, 0.6);

        top.Top_Pivot.setPosition(0.5);

        DropPreLoad();

        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bottom.Destacker_Left.setPosition(0.37);
        bottom.Destacker_Right.setPosition(0.37);
        Destack();
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bottom.Destacker_Left.setPosition(0.44);
        bottom.Destacker_Right.setPosition(0.44);
        Destack();
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bottom.Destacker_Left.setPosition(0.51);
        bottom.Destacker_Right.setPosition(0.51);
        Destack();
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bottom.Destacker_Left.setPosition(0.64);
        bottom.Destacker_Right.setPosition(0.64);
        Destack();
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bottom.Destacker_Left.setPosition(0.85);
        bottom.Destacker_Right.setPosition(0.85);
        Destack();

    }

    public void DropPreLoad(){
        //Extend vertical slides and drop cone
        slide.Right_Slide.setTargetPosition(2000);
        slide.Left_Slide.setTargetPosition(2000);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);
        }
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);

        top.Top_Pivot.setPosition(0);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(400);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        top.Top_Gripper.setPosition(0.3);

        //TO DO: Insert WHILE loop
        if(top.Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
            top.Top_Pivot.setPosition(0.4);
            slide.Right_Slide.setTargetPosition(0);
            slide.Left_Slide.setTargetPosition(0);
            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
            lowering = true;
        }
    }

    public void CheckVSlidePos() {
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

    public void Destack() {
        top.Top_Pivot.setPosition(0.5);
        bottom.Base_Pivot.setPosition(0);
        slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Extend.setPower(-1);
        conefound = slide.colour.blue() > 130;

        while(!conefound && slide.Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = slide.colour.blue() > 130;
            slide.Extend.setPower(-1);
        }
        slide.Extend.setPower(0);
        if(conefound) {
            //close gripper
            bottom.Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            top.Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            bottom.Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(bottom.Base_Gripper.getPosition() == 0){
                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (slide.Extend.isBusy()) {
                    CheckVSlidePos();
                    bottom.Base_Pivot.setPosition(1);
                    slide.Extend.setPower(1);
                }

                slide.Extend.setPower(0);

                //bring destacker down
                bottom.Destacker_Left.setPosition(1);
                bottom.Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                bottom.Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                top.Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                top.Top_Pivot.setPosition(1);


                if(bottom.Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    top.Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    top.Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    bottom.Base_Pivot.setPosition(0);

                }

            }

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            bottom.Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            slide.Right_Slide.setTargetPosition(2000);
            slide.Left_Slide.setTargetPosition(2000);
            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){
                slide.Right_Slide.setPower(1);
                slide.Left_Slide.setPower(1);
            }
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            top.Top_Pivot.setPosition(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            top.Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(top.Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                top.Top_Pivot.setPosition(0.4);
                slide.Right_Slide.setTargetPosition(0);
                slide.Left_Slide.setTargetPosition(0);
                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.Right_Slide.setPower(-0.9);
                slide.Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            top.Top_Pivot.setPosition(0.5);
            slide.Extend.setTargetPosition(0);
            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slide.Extend.isBusy()) {
                CheckVSlidePos();
                slide.Extend.setPower(0.8);
            }
            slide.Extend.setPower(0);
            bottom.Base_Pivot.setPosition(0);
        }


    }
}
