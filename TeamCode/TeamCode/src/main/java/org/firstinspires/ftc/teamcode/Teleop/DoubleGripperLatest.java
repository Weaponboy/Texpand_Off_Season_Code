package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DoubleGripperLatest extends OpMode {

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
    private double Destack_position = 0;

    private boolean conefound = false;
    private double slow = 1;
    private boolean lowering = false;



    @Override
    public void loop() {
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(slow*(-pivot + (vertical - horizontal)));
        RB.setPower(1.18*(slow*(-pivot + (vertical + horizontal))));
        LF.setPower(slow*(pivot + (vertical + horizontal)));
        LB.setPower(1.18*(slow*(pivot + (vertical - horizontal))));

        //destack
        if (gamepad1.dpad_up) {
            stakerpos = stakerpos + 1;

            if(stakerpos == 1){
                Destack_position = 1;
            } else if(stakerpos == 2) {
                Destack_position = 0.7;
            } else if(stakerpos == 3) {
                Destack_position = 0.65;
            } else if(stakerpos == 4) {
                Destack_position = 0.55;
            } else if(stakerpos == 5) {
                Destack_position = 0.47;
            }
            if(stakerpos > 5){
                stakerpos = 0;
            }
            Destacker_Left.setPosition(Destack_position);
            Destacker_Right.setPosition(Destack_position);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //Stop slides if finished running
        if(gamepad1.back){
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
                Top_Pivot.setPosition(0.77);
                Top_Gripper.setPosition(0.3);
                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                if(Base_Gripper.getPosition() == 0){
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Extend.isBusy()) {
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
                        Base_Gripper.setPosition(0);
                        Destacker_Left.setPosition(1);
                        Destacker_Right.setPosition(1);
                        Base_Pivot.setPosition(1);
                        Extend.setPower(1);
                    }
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
                    Extend.setPower(0);
                    if(Base_Pivot.getPosition() > 0.9) {
                        try {
                            Thread.sleep(125);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Top_Pivot.setPosition(1);
                        Base_Gripper.setPosition(0.4);
                        try {
                            Thread.sleep(250);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Top_Gripper.setPosition(0);
                    }

                }

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
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
                    Extend.setPower(0.8);
                }
                Extend.setPower(0);
            }
        }
        //Reduce robot speed

        if(gamepad1.start && slow == 1){
            slow = 0.4;
        }else if(gamepad1.start && slow < 1){
            slow = 1;
        }
        //toggle possition of base pivot

        if(gamepad1.a && Base_Pivot.getPosition() != 1){
            Base_Pivot.setPosition(1); //open base gripper if it is closed
        }else if(gamepad1.a && Base_Pivot.getPosition() != 0.35 ){
            Base_Pivot.setPosition(0); //close base gripper if it is open
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //sleep
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //toggle possiton of base gripper
        if(gamepad1.b && Base_Gripper.getPosition() == 0){
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(gamepad1.b && Base_Gripper.getPosition() > 0){
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        //toggle possition of top pivot
        if (gamepad1.x) {
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(0.75);
            }else if(Toppos == 2){
                Top_Pivot.setPosition(1);
            }
            if(Toppos > 2){
                Toppos = 0;
            }

        }

        //set top pivot postition
        if(gamepad1.dpad_down){
            Top_Pivot.setPosition(0.3);
        }
        //Sleep
        try {
            Thread.sleep(25);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        //toggle positioin of top gripper
        if(gamepad1.y && Top_Gripper.getPosition() == 0){
            Top_Gripper.setPosition(0.3); //lift up top griper if it is down
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }else if(gamepad1.y && Top_Gripper.getPosition() > 0){
            Top_Gripper.setPosition(0); //lower top gripper if it is up
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }

        //toggle lift slide power
        if(gamepad1.left_bumper){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }else if(gamepad1.right_bumper){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }else if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }

        //set slides to High pole
        if(gamepad1.dpad_left){
            Right_Slide.setTargetPosition(1900);
            Left_Slide.setTargetPosition(1900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            Top_Pivot.setPosition(0.3);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right){
            Right_Slide.setTargetPosition(900);
            Left_Slide.setTargetPosition(900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            Top_Pivot.setPosition(0.3);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }
        //Sleep
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }
        //extend slides to collect cone
        if (gamepad1.left_trigger > 0) {
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
                Top_Pivot.setPosition(0.77);
                Top_Gripper.setPosition(0.3);
                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                if(Base_Gripper.getPosition() == 0){
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Extend.isBusy()) {
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
                        Base_Gripper.setPosition(0);
                        Destacker_Left.setPosition(1);
                        Destacker_Right.setPosition(1);
                        Base_Pivot.setPosition(1);
                        Extend.setPower(1);
                    }
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
                    Extend.setPower(0);
                    if(Base_Pivot.getPosition() > 0.9) {
                        try {
                            Thread.sleep(125);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Top_Pivot.setPosition(1);
                        Base_Gripper.setPosition(0.4);
                        try {
                            Thread.sleep(250);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Top_Gripper.setPosition(0);
                        Top_Pivot.setPosition(0.4);
                        try {
                            Thread.sleep(75);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Base_Pivot.setPosition(0);
                    }

                }

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
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
                    Extend.setPower(0.8);
                }
                Extend.setPower(0);
            }
        }

//bring slides back to bottom
        if(gamepad1.right_trigger > 0){
            Top_Gripper.setPosition(0.3);
            if(Top_Gripper.getPosition() == 0.3) {
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Top_Pivot.setPosition(0.77);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }
        }
//stop slides if finished
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

            telemetry.addData("Destacker Left:", Destacker_Left.getPosition());
            telemetry.addData("Destacker Right:", Destacker_Right.getPosition());
            telemetry.addData("Base Pivot:", Base_Pivot.getPosition());
            telemetry.addData("Stacker pos:", stakerpos);
            telemetry.addData("Blue:", colour.blue());
            telemetry.addData("RF Power:", RF.getPower());
            telemetry.addData("RB Power:", RB.getPower());
            telemetry.addData("LF Power:", LF.getPower());
            telemetry.addData("LB Power:", LB.getPower());
            telemetry.update();


    }

    //init
    @Override
    public void init() {
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
        Destacker_Left = hardwareMap.get(Servo.class,"Destacker Left");
        Destacker_Right = hardwareMap.get(Servo.class,"Destacker Right");

        Destacker_Left.setDirection(Servo.Direction.REVERSE);
        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.FORWARD);

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
        Base_Pivot.setPosition(0);
        Base_Gripper.setPosition(0.4);


        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }
}