package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DoubleGripperLatest extends OpMode {

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    double De_Pos_1 = 0.0;
    double De_Pos_2 = 0.15;
    double De_Pos_3 = 0.55;
    double De_Pos_4 = 0.7;
    double De_Pos_5 = 0.8;

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

    private DistanceSensor sensorRange;

    private boolean Nest_Occupied = false;

    private double vertical;
    private double horizontal;
    private double pivot;

    private double Base_Pivot_Collect = 0.08;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;

    private double Top_Pivot_Collect = 0.33;

    private double Top_Gripper_Collect_Wide = 0.36;

    private boolean rumble = false;

    private int Toppos = 0;
    private int stakerpos = 0;
    private double Destack_position = 0;

    private boolean conefound = false;
    private boolean conefoundDe = false;
    private double slow = 0.4;
    private boolean lowering = false;

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void loop() {
        runtime.reset();

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(slow*(-pivot + (vertical - horizontal)));
        RB.setPower(1.18*(slow*(-pivot + (vertical + horizontal))));
        LF.setPower(slow*(pivot + (vertical + horizontal)));
        LB.setPower(1.18*(slow*(pivot + (vertical - horizontal))));


        //destack
        if (gamepad2.dpad_up) {
            stakerpos = stakerpos + 1;

            if(stakerpos == 1){
                Destack_position = De_Pos_1;
                Base_Pivot.setPosition(0.12);
            } else if(stakerpos == 2) {
                Destack_position = De_Pos_2;
                Base_Pivot.setPosition(Base_Pivot_Collect);
            } else if(stakerpos == 3) {
                Destack_position = De_Pos_3;
                Base_Pivot.setPosition(Base_Pivot_Collect);
            } else if(stakerpos == 4) {
                Destack_position = De_Pos_4;
                Base_Pivot.setPosition(Base_Pivot_Collect);
            } else if(stakerpos == 5) {
                Destack_position = De_Pos_5;
                Base_Pivot.setPosition(Base_Pivot_Collect);
            }
            if(stakerpos > 5){
                stakerpos = 0;
            }
            Destacker_Left.setPosition(Destack_position);
            Destacker_Right.setPosition(Destack_position);
        }

        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed

        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        if (gamepad1.dpad_up) {
            Destacker_Left.setPosition(De_Pos_1);
            Destacker_Right.setPosition(De_Pos_1);
            Base_Pivot.setPosition(0.12);
        }
        if (gamepad1.right_bumper) {
            Destacker_Left.setPosition(De_Pos_2);
            Destacker_Right.setPosition(De_Pos_2);
        }
        if (gamepad1.y) {
            Destacker_Left.setPosition(De_Pos_3);
            Destacker_Right.setPosition(De_Pos_3);
        }
        if (gamepad1.b) {
            Destacker_Left.setPosition(De_Pos_4);
            Destacker_Right.setPosition(De_Pos_4);
        }
        if (gamepad1.a) {
            Destacker_Left.setPosition(De_Pos_5);
            Destacker_Right.setPosition(De_Pos_5);
        }


        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

//        if(!(sensorRange.getDistance(DistanceUnit.CM) > 6 && sensorRange.getDistance(DistanceUnit.CM) < 50 && Base_Pivot.getPosition() < 0.5) && rumble){
//            rumble = false;
//        }
//        if (sensorRange.getDistance(DistanceUnit.CM) > 6 && sensorRange.getDistance(DistanceUnit.CM) < 50 && Base_Pivot.getPosition() < 0.5 && !rumble){
//            gamepad1.runRumbleEffect(customRumbleEffect);
//            gamepad2.runRumbleEffect(customRumbleEffect);
//
//            rumble = true;
//        }


//        public void Destack(double De_pos) {
//
//            Base_Gripper.setPosition(0.4);
//
//            Destacker_Left.setPosition(De_pos);
//            Destacker_Right.setPosition(De_pos);
//
//            try {
//                Thread.sleep(100);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//
//            Top_Pivot.setPosition(0.5);
//            Base_Pivot.setPosition(0.1);
//
//            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            Extend.setPower(-1);
//
//            conefound = sensorRange.getDistance(DistanceUnit.MM) < 30;
//
//            //extend till we find a cone or get to the slides limit
//            while (!conefound && Extend.getCurrentPosition() > -1930) {
//
//                CheckVSlidePos();
//
//                conefound = sensorRange.getDistance(DistanceUnit.MM) < 30;
//
//                Extend.setPower(-1);
//
//            }
//
//            Extend.setPower(0);
//
//            if (conefound) {
//
//                //close gripper
//                Base_Gripper.setPosition(0);
//
//                CheckVSlidePos();
//
//                //make sure gripper is closed
//                try {
//                    Thread.sleep(100);
//                } catch (Exception e) {
//                    System.out.println(e.getMessage());
//                }
//
//                //if the base gripper is closed retract the horizontal slides
//                if (Base_Gripper.getPosition() == 0) {
//
//                    Extend.setTargetPosition(0);
//                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    while (Extend.isBusy()) {
//                        CheckVSlidePos();
//                        Base_Pivot.setPosition(0.9);
//                        Extend.setPower(1);
//                    }
//
//                    Extend.setPower(0);
//
//                    //bring destacker down
//                    Destacker_Left.setPosition(1);
//                    Destacker_Right.setPosition(1);
//
//                    while (lowering) {
//                        CheckVSlidePos();
//                    }
//
//                    //open base gripper
//                    Base_Gripper.setPosition(0.4);
//
//                    Nest_Occupied = colour.blue() > 3000;
//
//                    if (Nest_Occupied) {
//                        //open top gripper
//                        Top_Gripper.setPosition(0.35);
//
//                        try {
//                            Thread.sleep(400);
//                        } catch (Exception e) {
//                            System.out.println(e.getMessage());
//                        }
//
//                        //take top pivot to pick up the cone
//                        Top_Pivot.setPosition(1);
//
//                        try {
//                            Thread.sleep(400);
//                        } catch (Exception e) {
//                            System.out.println(e.getMessage());
//                        }
//
//
//                        if (Base_Pivot.getPosition() > 0.9) {
//
//                            try {
//                                Thread.sleep(200);
//                            } catch (Exception e) {
//                                System.out.println(e.getMessage());
//                            }
//
//                            //close top gripper
//                            Top_Gripper.setPosition(0);
//
//                            try {
//                                Thread.sleep(120);
//                            } catch (Exception e) {
//                                System.out.println(e.getMessage());
//                            }
//
//                            //take top pivot over
//                            Top_Pivot.setPosition(0.5);
//
//                            try {
//                                Thread.sleep(25);
//                            } catch (Exception e) {
//                                System.out.println(e.getMessage());
//                            }
//
//                            //put base pivot back to zero
//                            Base_Pivot.setPosition(0.1);
//
//                        }
//
//                    }else {
//                        //Abort
//                        abort = true;
//                    }
//
//                    Nest_Occupied = colour.blue() > 3000;
//
//                    if(!Nest_Occupied){
//                        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                        Base_Pivot.setPosition(0.1);
//
//                        //Extend vertical slides and drop cone
//                        Right_Slide.setTargetPosition(2000);
//                        Left_Slide.setTargetPosition(2000);
//                        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        while (Right_Slide.isBusy() && Left_Slide.isBusy()) {
//                            Right_Slide.setPower(1);
//                            Left_Slide.setPower(1);
//                        }
//                        Right_Slide.setPower(0);
//                        Left_Slide.setPower(0);
//
//                        Top_Pivot.setPosition(0);
//
//                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        try {
//                            Thread.sleep(200);
//                        } catch (Exception e) {
//                            System.out.println(e.getMessage());
//                        }
//
//                        Top_Gripper.setPosition(0.3);
//
//                        try {
//                            Thread.sleep(100);
//                        } catch (Exception e) {
//                            System.out.println(e.getMessage());
//                        }
//
//                        //TO DO: Insert WHILE loop
//                        if (Top_Gripper.getPosition() == 0.3) {
//                            try {
//                                Thread.sleep(250);
//                            } catch (Exception e) {
//                                System.out.println(e.getMessage());
//                            }
//                            Top_Pivot.setPosition(0.4);
//                            Right_Slide.setTargetPosition(0);
//                            Left_Slide.setTargetPosition(0);
//                            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                            Right_Slide.setPower(-0.9);
//                            Left_Slide.setPower(-0.9);
//                            lowering = true;
//
//                        }
//
//                    }else {
//                        //Abort
//                        abort = true;
//                    }
//
//                }
//
//            }else {
//                abort = true;
//                Top_Pivot.setPosition(0.5);
//                Extend.setTargetPosition(0);
//                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while (Extend.isBusy()) {
//                    CheckVSlidePos();
//                    Extend.setPower(0.8);
//                }
//                Extend.setPower(0);
//                Base_Pivot.setPosition(0.1);
//            }
//            try {
//                Thread.sleep(20);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//
//        }

        //Stop slides if finished running

        if(gamepad1.back || gamepad2.back){

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);

            conefound = sensorRange.getDistance(DistanceUnit.MM) < 50;


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
                conefound = sensorRange.getDistance(DistanceUnit.MM) < 65;
                Extend.setPower(-1);
                try {
                    Thread.sleep(20);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                telemetry.addData("Blue:",sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            Extend.setPower(0);

            if(conefound) {
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
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

                Top_Pivot.setPosition(0.6);

                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                Base_Pivot.setPosition(Base_Pivot_Flip);

                if (Destacker_Left.getPosition() < 0.6){

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

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Extend.setPower(1);

//                        conefoundDe = colour.blue() > 3000;
//
//                        if (conefoundDe || Extend.getCurrentPosition() < -700){
//                            Base_Gripper.setPosition(0.4);
//
//                            Destacker_Left.setPosition(De_Pos_5);
//                            Destacker_Right.setPosition(De_Pos_5);
//                        }
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

                    Destacker_Left.setPosition(0.8);
                    Destacker_Right.setPosition(0.8);

                    if(Base_Pivot.getPosition() > 0.7) {

                        try {
                            Thread.sleep(200);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(1);

                        Base_Gripper.setPosition(0.4);

                        Base_Pivot.setPosition(Base_Pivot_Out_Way);

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

        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        //Reduce robot speed
        if(gamepad1.start && slow == 1){
            slow = 0.4;
        }else if(gamepad1.start && slow < 1){
            slow = 1;
        }

        //toggle possition of base pivot
//        if(gamepad2.a && Base_Pivot.getPosition() != 0.85){
//            try {
//                Thread.sleep(50);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//            Base_Pivot.setPosition(0.85); //open base gripper if it is closed
//        }else if(gamepad2.a && Base_Pivot.getPosition() != Base_Pivot_Collect ){
//            try {
//                Thread.sleep(50);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//            Base_Pivot.setPosition(Base_Pivot_Collect); //close base gripper if it is open
//        }

        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        //toggle possiton of base gripper


        //toggle possition of top pivot
        if (gamepad2.x || gamepad1.x) {
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(0.6);
            }else if(Toppos == 2){
                Top_Pivot.setPosition(1);
            }
            if(Toppos > 2){
                Toppos = 0;
            }

        }

        //set top pivot postition
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Top_Pivot.setPosition(0);
        }


        //toggle positioin of top gripper
        if(gamepad2.y && Top_Gripper.getPosition() == 0){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Top_Gripper.setPosition(Top_Pivot_Collect); //lift up top griper if it is down
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }else if(gamepad2.y && Top_Gripper.getPosition() > 0){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Top_Gripper.setPosition(0); //lower top gripper if it is up
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }

        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper){


                Base_Pivot.setPosition(Base_Pivot_Collect);

                RF.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                LB.setPower(0);

                Base_Pivot.setPosition(Base_Pivot_Collect);

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Extend.setPower(-1);
                conefound = sensorRange.getDistance(DistanceUnit.MM) < 50;

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
                    conefound = sensorRange.getDistance(DistanceUnit.MM) < 65;

                    Extend.setPower(-1);

                    telemetry.addData("Blue:", sensorRange.getDistance(DistanceUnit.MM));
                    telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                    telemetry.addData("Cone found:", conefound);
                    telemetry.update();
                }

                Extend.setPower(0);

                if(conefound) {

                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Base_Gripper.setPosition(0);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
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

                    Top_Pivot.setPosition(0.77);
                    Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

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

                            Base_Pivot.setPosition(Base_Pivot_Flip);

                            Top_Pivot.setPosition(0.6);

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

                        Destacker_Left.setPosition(0.8);
                        Destacker_Right.setPosition(0.8);

                        if(Base_Pivot.getPosition() > 0.6) {

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Base_Gripper.setPosition(0.4);

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Base_Pivot.setPosition(Base_Pivot_Out_Way);

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(1);

                            try {
                                Thread.sleep(250);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Gripper.setPosition(0);

                            try {
                                Thread.sleep(150);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(0.4);

                            try {
                                Thread.sleep(75);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }
                            Base_Pivot.setPosition(Base_Pivot_Collect);
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
                    Base_Pivot.setPosition(Base_Pivot_Collect);
                }

            try {
                Thread.sleep(200);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

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

            Top_Pivot.setPosition(0.1);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            try {
                Thread.sleep(400);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(Top_Pivot_Collect);
            if(Top_Gripper.getPosition() == Top_Pivot_Collect) {

                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Top_Pivot.setPosition(0.6);

                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);

                lowering = true;
            }

        }

        //set slides to High pole
        if(gamepad1.dpad_left || gamepad2.dpad_left){
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

            Top_Pivot.setPosition(0.1);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gamepad2.a && Base_Pivot.getPosition() != 0.85){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Pivot.setPosition(0.85); //open base gripper if it is closed
        }else if(gamepad2.a && Base_Pivot.getPosition() != Base_Pivot_Collect ){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Pivot.setPosition(Base_Pivot_Collect); //close base gripper if it is open
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right || gamepad2.dpad_right){
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

            Top_Pivot.setPosition(0.1);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //extend slides to collect cone
        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {

            Base_Pivot.setPosition(Base_Pivot_Collect);

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Base_Pivot.setPosition(Base_Pivot_Collect);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);
            conefound = sensorRange.getDistance(DistanceUnit.MM) < 65;

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
                conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

                Extend.setPower(-1);

                try {
                    Thread.sleep(10);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                telemetry.addData("Blue:", sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            Extend.setPower(0);

            if(conefound) {

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
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

                Top_Pivot.setPosition(0.77);
                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

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

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Top_Pivot.setPosition(0.6);

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

                    Destacker_Left.setPosition(0.8);
                    Destacker_Right.setPosition(0.8);

                    if(Base_Pivot.getPosition() > 0.6) {

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Base_Gripper.setPosition(0.4);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(1);

                        try {
                            Thread.sleep(150);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(150);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(0.4);

                        try {
                            Thread.sleep(75);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        Base_Pivot.setPosition(Base_Pivot_Collect);
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
                Base_Pivot.setPosition(Base_Pivot_Collect);
            }
        }

//bring slides back to bottom
        if(gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0){
            Top_Gripper.setPosition(Top_Pivot_Collect);
            if(Top_Gripper.getPosition() == Top_Pivot_Collect) {

                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Top_Pivot.setPosition(0.6);

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


        telemetry.addData("Rumble:", rumble);
        telemetry.addData("Destacker Left:", Destacker_Left.getPosition());
        telemetry.addData("Destacker Right:", Destacker_Right.getPosition());
        telemetry.addData("Base Pivot:", Base_Pivot.getPosition());
        telemetry.addData("Stacker pos:", stakerpos);
        telemetry.addData("MM range:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("Blue:", colour.blue());
        telemetry.addData("RF Power:", RF.getPower());
        telemetry.addData("RB Power:", RB.getPower());
        telemetry.addData("LF Power:", LF.getPower());
        telemetry.addData("LB Power:", LB.getPower());
        telemetry.addData("motor ticks extend:", Extend.getCurrentPosition());

        telemetry.update();

    }

    //init
    @Override
    public void init() {
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        colour = hardwareMap.get(ColorSensor.class, "colour");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

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
//        Base_Pivot.setPosition(Base_Pivot_Collect);
        Base_Gripper.setPosition(0.4);

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();


        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }

}