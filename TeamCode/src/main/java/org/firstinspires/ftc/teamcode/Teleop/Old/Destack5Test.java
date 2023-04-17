package org.firstinspires.ftc.teamcode.Teleop.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous
public class Destack5Test extends LinearOpMode {

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    public ColorSensor colour = null;

    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;

    public DcMotor Extend = null;

    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;
    public Servo Top_Gripper = null;
    public Servo Top_Pivot = null;
    public Servo Destacker_Left = null;
    public Servo Destacker_Right = null;

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

    @Override
    public void runOpMode() throws InterruptedException {

        colour = hardwareMap.get(ColorSensor.class, "colour");

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Base_Gripper = hardwareMap.get(Servo.class, "Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class, "Base_Pivot");
        Top_Gripper = hardwareMap.get(Servo.class, "Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class, "Top_Pivot");
        Destacker_Left = hardwareMap.get(Servo.class, "Destacker Left");
        Destacker_Right = hardwareMap.get(Servo.class, "Destacker Right");

        Destacker_Left.setDirection(Servo.Direction.REVERSE);
        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.REVERSE);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        Top_Gripper.setPosition(0.3);
        Base_Gripper.setPosition(0.4);
        Base_Pivot.setPosition(0);
        Top_Pivot.setPosition(0.4);

        waitForStart();

        //top cone
        Destacker_Left.setPosition(0.4);
        Destacker_Right.setPosition(0.4);
        Destack_position = 0.4;
        Destack();

//        //4th cone
//        Destacker_Left.setPosition(0.55);
//        Destacker_Right.setPosition(0.55);
//        Destack_position = 0.55;
//        Destack();
//
//        //3rd cone
//        Destacker_Left.setPosition(0.65);
//        Destacker_Right.setPosition(0.65);
//        Destack_position = 0.65;
//        Destack();
//
//        //2nd cone
//        Destacker_Left.setPosition(0.7);
//        Destacker_Right.setPosition(0.7);
//        Destack_position = 0.7;
//        Destack();
//
//        //1st cone
//        Destacker_Left.setPosition(1);
//        Destacker_Right.setPosition(1);
//        Destack_position = 1;
//        Destack();
        while (lowering) {
            CheckVSlidePos();
        }

    }
    public void CheckVSlidePos() {
        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;
        }else if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }
    }

    public void Destack() {
        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(400);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(300);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(300);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(300);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(400);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(100);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.75);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }


    }

//    public void init(HardwareMap hwMap) {
//
//        hardwareMap = hwMap;
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
//
//        RF.setPower(0);
//        LF.setPower(0);
//        RB.setPower(0);
//        LB.setPower(0);
//        Top_Gripper.setPosition(0.3);
//        Base_Gripper.setPosition(0.4);
//        Base_Pivot.setPosition(0);
//
//
//    }
}