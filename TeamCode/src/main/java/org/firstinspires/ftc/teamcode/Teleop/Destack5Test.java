package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    public Servo Destacker = null;

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

        Base_Gripper = hardwareMap.get(Servo.class,"Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class,"Base_Pivot");
        Top_Gripper = hardwareMap.get(Servo.class,"Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class,"Top_Pivot");
        Destacker = hardwareMap.get(Servo.class,"Destacker");

        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.FORWARD);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

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
        Top_Gripper.setPosition(0.4);
        Base_Pivot.setPosition(0.35);
        Base_Gripper.setPosition(0.45);
        Destacker.setPosition(0.2);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();
        //top cone
        Destacker.setPosition(0.6);
        basepos = 0.1;
        Base_Pivot.setPosition(0.0);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        Extend.setPower(0);
        if(conefound) {
            Base_Gripper.setPosition(0);
            Top_Pivot.setPosition(1);
            Top_Gripper.setPosition(0.45);

            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }

            if(Base_Gripper.getPosition() <= 0.01){
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(1);
                while(Base_Pivot.getPosition() < 0.95){
                    try {
                        Thread.sleep(10);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    telemetry.addData("base pivot", Base_Pivot.getPosition());
                    telemetry.update();

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(1);
                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Destacker.setPosition(0.2);

                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
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

                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    Top_Gripper.setPosition(0);
                    if (Top_Gripper.getPosition() <= 0.01) {
                        Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        telemetry.addData("got", "here");
                        telemetry.update();
                        Top_Pivot.setPosition(0.3);
                        Base_Pivot.setPosition(0.35);

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }
        Base_Pivot.setPosition(0.35);
        Right_Slide.setTargetPosition(1900);
        Left_Slide.setTargetPosition(1900);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(700);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        Top_Gripper.setPosition(0.45);
        if(Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }
        //4th cone
        Top_Gripper.setPosition(0.45);

        Destacker.setPosition(0.52);
        basepos = 0.12;
        Base_Pivot.setPosition(0.14);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        Extend.setPower(0);
        if(conefound) {
            Base_Gripper.setPosition(0);
            Top_Pivot.setPosition(1);
            Top_Gripper.setPosition(0.45);

            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }

            if(Base_Gripper.getPosition() <= 0.01){
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(1);
                while(Base_Pivot.getPosition() < 0.95){
                    try {
                        Thread.sleep(10);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    telemetry.addData("base pivot", Base_Pivot.getPosition());
                    telemetry.update();

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(1);
                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Destacker.setPosition(0.2);

                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
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

                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    Top_Gripper.setPosition(0);
                    if (Top_Gripper.getPosition() <= 0.01) {
                        Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        telemetry.addData("got", "here");
                        telemetry.update();
                        Top_Pivot.setPosition(0.3);
                        Base_Pivot.setPosition(0.35);

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }
        Base_Pivot.setPosition(0.35);
        Right_Slide.setTargetPosition(1900);
        Left_Slide.setTargetPosition(1900);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(700);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        Top_Gripper.setPosition(0.45);
        if(Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }
        //3rd cone
        Top_Gripper.setPosition(0.45);

        Destacker.setPosition(0.45);
        basepos = 0.22;
        Base_Pivot.setPosition(0.23);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        Extend.setPower(0);
        if(conefound) {
            Base_Gripper.setPosition(0);
            Top_Pivot.setPosition(1);
            Top_Gripper.setPosition(0.45);

            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }

            if(Base_Gripper.getPosition() <= 0.01){
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(1);
                while(Base_Pivot.getPosition() < 0.95){
                    try {
                        Thread.sleep(10);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    telemetry.addData("base pivot", Base_Pivot.getPosition());
                    telemetry.update();

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(1);
                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Destacker.setPosition(0.2);

                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
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

                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    Top_Gripper.setPosition(0);
                    if (Top_Gripper.getPosition() <= 0.01) {
                        Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        telemetry.addData("got", "here");
                        telemetry.update();
                        Top_Pivot.setPosition(0.3);
                        Base_Pivot.setPosition(0.35);

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }
        Base_Pivot.setPosition(0.35);
        Right_Slide.setTargetPosition(1900);
        Left_Slide.setTargetPosition(1900);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(700);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        Top_Gripper.setPosition(0.45);
        if(Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }
        //2nd cone
        Top_Gripper.setPosition(0.45);

        Destacker.setPosition(0.4);
        basepos = 0.28;
        Base_Pivot.setPosition(0.28);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        Extend.setPower(0);
        if(conefound) {
            Base_Gripper.setPosition(0);
            Top_Pivot.setPosition(1);
            Top_Gripper.setPosition(0.45);

            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }

            if(Base_Gripper.getPosition() <= 0.01){
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(1);
                while(Base_Pivot.getPosition() < 0.95){
                    try {
                        Thread.sleep(10);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    telemetry.addData("base pivot", Base_Pivot.getPosition());
                    telemetry.update();

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(1);
                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Destacker.setPosition(0.2);

                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
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

                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    Top_Gripper.setPosition(0);
                    if (Top_Gripper.getPosition() <= 0.01) {
                        Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        telemetry.addData("got", "here");
                        telemetry.update();
                        Top_Pivot.setPosition(0.3);
                        Base_Pivot.setPosition(0.35);

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }
        Base_Pivot.setPosition(0.35);
        Right_Slide.setTargetPosition(1900);
        Left_Slide.setTargetPosition(1900);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(700);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        Top_Gripper.setPosition(0.45);
        if(Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }
        //1st cone
        Top_Gripper.setPosition(0.45);

        Destacker.setPosition(0.2);
        basepos = 0.35;
        Base_Pivot.setPosition(0.35);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("motor ticks:", Extend.getCurrentPosition());
            telemetry.addData("Cone found:", conefound);
            telemetry.update();
        }

        Extend.setPower(0);
        if(conefound) {
            Base_Gripper.setPosition(0);
            Top_Pivot.setPosition(1);
            Top_Gripper.setPosition(0.45);

            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }

            if(Base_Gripper.getPosition() <= 0.01){
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(1);
                while(Base_Pivot.getPosition() < 0.95){
                    try {
                        Thread.sleep(10);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    telemetry.addData("base pivot", Base_Pivot.getPosition());
                    telemetry.update();

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Extend.setPower(1);
                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Destacker.setPosition(0.2);

                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
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

                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    Top_Gripper.setPosition(0);
                    if (Top_Gripper.getPosition() <= 0.01) {
                        Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        telemetry.addData("got", "here");
                        telemetry.update();
                        Top_Pivot.setPosition(0.3);
                        Base_Pivot.setPosition(0.35);

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }
        Base_Pivot.setPosition(0.35);
        Right_Slide.setTargetPosition(1900);
        Left_Slide.setTargetPosition(1900);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(700);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        Top_Gripper.setPosition(0.45);
        if(Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }

    }

}
