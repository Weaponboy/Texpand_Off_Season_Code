package org.firstinspires.ftc.teamcode.Teleop.Test_Class;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Cone_Stack_Vision;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.concurrent.TimeUnit;

@TeleOp
public class DoubleGripperLatestCamera extends OpMode {

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    private double power;

    private OpenCvWebcam BackWeb;

    private OpenCvWebcam frontWeb;


    public DcMotor Odo_raise  = null;

    private double TotalDist;
    private double AveDist;
    Drivetrain drive = new Drivetrain();

    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();

    Pole_Pipe Pole;

    Blue_Cone_Pipe Cone;

    double De_Pos_1 = 0.0;
    double De_Pos_2 = 0.2;
    double De_Pos_3 = 0.45;
    double De_Pos_4 = 0.54;
    double De_Pos_5 = 0.7;

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

    private DistanceSensor Back_Distance;

    private boolean Nest_Occupied = false;

    private double vertical;
    private double horizontal;
    private double pivot;

    private double Base_Pivot_Collect = 0.1;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;

    private double Top_Pivot_Collect = 0.31;

    private double Top_Gripper_Collect_Wide = 0.36;

    private boolean rumble = false;

    private int Toppos = 0;
    private int stakerpos = 0;
    private double Destack_position = 0;


    private  double poleDistance = 0;

    private  double TargetPoleDistance = 420;

    private  double TravelDistance = 0;

    private  int Loop = 0;
    private  int Loop2 = 0;

    public double Distance_To_Travel;


    public double ConversionPixelstoCm = 22;//need to tune this

    public double CenterOfScreen = 320;

    public double rectPositionFromLeft = 0;

    private boolean conefound = false;

    private boolean SlowPoint = false;
    private double slow = 0.4;

    private double slow1 = 0.4;

    private boolean lowering = false;

    @Override
    public void loop() {
        runtime.reset();
        slow1 = (gamepad1.left_trigger * 0.6) + 0.4;

        drive.WithOutEncoders();

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(slow1*1.3*(-pivot + (vertical - horizontal)));
        RB.setPower(slow1*(-pivot + (vertical + horizontal)));
        LF.setPower(slow1*1.3*(pivot + (vertical + horizontal)));
        LB.setPower(slow1*(pivot + (vertical - horizontal)));



        if (gamepad2.dpad_up && gamepad2.back) {
            stakerpos = stakerpos + 1;

            if(stakerpos == 1 && Odo_raise.getCurrentPosition() < 50){
                Pods_Up();
            } else if(stakerpos == 2 && Odo_raise.getCurrentPosition() > 50) {
                Pods_Down();
            }

            if(stakerpos > 2){
                stakerpos = 0;
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

        if (gamepad1.dpad_up) {

        }
        if (gamepad1.right_bumper) {
            Destacker_Left.setPosition(De_Pos_2);
            Destacker_Right.setPosition(De_Pos_2);
            Base_Pivot.setPosition(0.1);
        }
        if (gamepad1.y) {
            Destacker_Left.setPosition(De_Pos_3);
            Destacker_Right.setPosition(De_Pos_3);
            Base_Pivot.setPosition(0.1);
        }
        if (gamepad1.b) {
            Destacker_Left.setPosition(De_Pos_4);
            Destacker_Right.setPosition(De_Pos_4);
            Base_Pivot.setPosition(0.1);
        }
        if (gamepad1.a) {
            Destacker_Left.setPosition(De_Pos_5);
            Destacker_Right.setPosition(De_Pos_5);
            Base_Pivot.setPosition(Base_Pivot_Collect);
        }


        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        if(gamepad2.back && !gamepad2.dpad_up){

            Base_Pivot.setPosition(Base_Pivot_Collect);

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);

            conefound = sensorRange.getDistance(DistanceUnit.MM) < 60;


            while(!conefound && Extend.getCurrentPosition() > -900){
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

                conefound = sensorRange.getDistance(DistanceUnit.MM) < 60;

                SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 180;

                if (SlowPoint){
                    Extend.setPower(-0.5);
                }else {
                    Extend.setPower(-1);
                }

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

                        Extend.setPower(0.8);

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
        if(gamepad1.start && slow == 0.6){
            slow = 0.4;
        }else if(gamepad1.start && slow < 0.6){
            slow = 0.6;
        }

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

        //toggle possition of top pivot
        if (gamepad2.x || gamepad1.x) {
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(0.22);
            }else if(Toppos == 2){
                Top_Pivot.setPosition(1);
            }else if(Toppos == 3){
                Top_Pivot.setPosition(0.6);
            }
            if(Toppos > 3){
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

        if (gamepad2.left_bumper || gamepad1.left_bumper){

                Base_Pivot.setPosition(Base_Pivot_Collect);

                RF.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                LB.setPower(0);

                Base_Pivot.setPosition(Base_Pivot_Collect);

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Extend.setPower(-1);
                conefound = sensorRange.getDistance(DistanceUnit.MM) < 90;

                while(!conefound && Extend.getCurrentPosition() > -900){
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
                    conefound = sensorRange.getDistance(DistanceUnit.MM) < 90;

                    SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 180;

                    if (SlowPoint){
                        Extend.setPower(-0.5);
                    }else {
                        Extend.setPower(-1);
                    }

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

                            frontWeb.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                                @Override
                                public void onClose() {
                                    BackWeb.stopStreaming();
                                }
                            });

                            BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                @Override
                                public void onOpened() {

                                    BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                                    BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                                    BackWeb.getGainControl().setGain(100);

                                    FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                                    BackWeb.getFocusControl().setMode(focusmode);

                                    if (focusmode == FocusControl.Mode.Fixed){
                                        BackWeb.getFocusControl().setFocusLength(450);
                                    }

                                    BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                                }

                                @Override
                                public void onError(int errorCode) { }
                            });

                            Base_Gripper.setPosition(0);

                            Base_Pivot.setPosition(Base_Pivot_Flip);

                            Top_Pivot.setPosition(0.6);

                            Extend.setPower(0.8);
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
                        Extend.setPower(0.5);
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

                Top_Pivot.setPosition(0.28);

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

        //Align to the top pole
        if(gamepad2.dpad_left || gamepad1.dpad_left){
            Right_Slide.setTargetPosition(1900);
            Left_Slide.setTargetPosition(1900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();
                power = 0.35;
                drive.WithOutEncoders();

                while (rectPositionFromLeft > CenterOfScreen + 10 || rectPositionFromLeft < CenterOfScreen - 10){

                    telemetry.addData("rect X", Pole.getRectX());
                    telemetry.addData("rect Y", Pole.getRectY());
                    telemetry.addData("Target CM", Distance_To_Travel);
                    telemetry.update();

                    if(rectPositionFromLeft > CenterOfScreen - 5 || rectPositionFromLeft < CenterOfScreen + 5){
                        power = 0.27;
                    }
                    rectPositionFromLeft = Pole.getRectX();

                    if (rectPositionFromLeft < CenterOfScreen) {

                        drive.RF.setPower(-1.3*power);
                        drive.RB.setPower(power);
                        drive.LF.setPower(1.3*power);
                        drive.LB.setPower(-power);

                    } else if (rectPositionFromLeft > CenterOfScreen) {

                        drive.RF.setPower(1.3*power);
                        drive.RB.setPower(-power);
                        drive.LF.setPower(-1.3*power);
                        drive.LB.setPower(power);
                    }

                }

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                drive.DriveEncoders();
            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            Top_Pivot.setPosition(0);

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

            Loop2 = 0;
            Loop = 0;

            Right_Slide.setTargetPosition(900);
            Left_Slide.setTargetPosition(900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();
                if(rectPositionFromLeft > CenterOfScreen + 15 || rectPositionFromLeft < CenterOfScreen - 15) {
                    if (rectPositionFromLeft < CenterOfScreen) {

                        drive.RF.setPower(-1.3 * 0.3);
                        drive.RB.setPower(-0.3);
                        drive.LF.setPower(1.3 * 0.3);
                        drive.LB.setPower(0.3);

                    } else if (rectPositionFromLeft > CenterOfScreen) {
                        drive.RF.setPower(1.3 * 0.3);
                        drive.RB.setPower(0.3);
                        drive.LF.setPower(-1.3 * 0.3);
                        drive.LB.setPower(-0.3);
                    } else {
                        drive.RF.setPower(0);
                        drive.RB.setPower(0);
                        drive.LF.setPower(0);
                        drive.LB.setPower(0);
                    }
                }else{
                    drive.RF.setPower(0);
                    drive.RB.setPower(0);
                    drive.LF.setPower(0);
                    drive.LB.setPower(0);
                }
                drive.WithOutEncoders();
            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            Top_Pivot.setPosition(0);

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
        if (gamepad2.left_trigger > 0 || gamepad1.back) {


            if(Destacker_Right.getPosition() > 0.7){
                Base_Pivot.setPosition(0.1);
            }else{
                Base_Pivot.setPosition(Base_Pivot_Collect);
            }

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Base_Pivot.setPosition(Base_Pivot_Collect);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);
            conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

            while(!conefound && Extend.getCurrentPosition() > -900){
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

                SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 180;

                if (SlowPoint){
                    Extend.setPower(-0.5);
                }else {
                    Extend.setPower(-1);
                }

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

                if(Right_Slide.getCurrentPosition() < 5 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 5 && !Left_Slide.isBusy()){
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
                        frontWeb.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                            @Override
                            public void onClose() {
                                BackWeb.stopStreaming();
                            }
                        });

                        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                            @Override
                            public void onOpened() {

                                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                                BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                                BackWeb.getGainControl().setGain(100);

                                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                                BackWeb.getFocusControl().setMode(focusmode);

                                if (focusmode == FocusControl.Mode.Fixed){
                                    BackWeb.getFocusControl().setFocusLength(450);
                                }

                                BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                            }

                            @Override
                            public void onError(int errorCode) { }
                        });

                        Base_Gripper.setPosition(0);

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Top_Pivot.setPosition(0.6);

                        Extend.setPower(0.8);
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

                    Destacker_Left.setPosition(De_Pos_5);
                    Destacker_Right.setPosition(De_Pos_5);

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
                            Thread.sleep(400);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(200);
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

                BackWeb.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                    @Override
                    public void onClose() {
                        BackWeb.stopStreaming();
                    }
                });

                frontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {

                        BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                        BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                        BackWeb.getGainControl().setGain(100);

                        FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                        BackWeb.getFocusControl().setMode(focusmode);

                        if (focusmode == FocusControl.Mode.Fixed){
                            BackWeb.getFocusControl().setFocusLength(450);
                        }

                        BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                    }

                    @Override
                    public void onError(int errorCode) { }
                });

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


        telemetry.addData("Camera front", frontWeb.getFps());
        telemetry.addData("Camera Back", BackWeb.getFps());
        telemetry.addData("Odo ticks:", Odo_raise.getCurrentPosition());
        telemetry.addData("Destacker Left:", Destacker_Left.getPosition());
        telemetry.addData("Destacker Right:", Destacker_Right.getPosition());
        telemetry.addData("Base Pivot:", Base_Pivot.getPosition());
        telemetry.addData("Stacker pos:", stakerpos);
        telemetry.addData("MM range:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("MM range:", Back_Distance.getDistance(DistanceUnit.MM));
        telemetry.addData("Blue:", colour.blue());
        telemetry.addData("RF Power:", RF.getPower());
        telemetry.addData("RB Power:", RB.getPower());
        telemetry.addData("LF Power:", LF.getPower());
        telemetry.addData("LB Power:", LB.getPower());
        telemetry.addData("motor ticks extend:", Extend.getCurrentPosition());
        telemetry.addData("Rect X:", Pole.getRectX());

        telemetry.update();

    }

    //init
    @Override
    public void init() {

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        Odo_raise = hardwareMap.get(DcMotor.class, "Odo_motor");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        colour = hardwareMap.get(ColorSensor.class, "colour");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

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

        Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        Odo_raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Odo_raise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        Top_Pivot.setPosition(0.55);
        Top_Gripper.setPosition(0.3);
        Base_Pivot.setPosition(Base_Pivot_Collect);
        Base_Gripper.setPosition(0.4);

        drive.init(hardwareMap);

        Pole = new Pole_Pipe();

        Cone = new Blue_Cone_Pipe();

        Back_Distance.resetDeviceConfigurationForOpMode();

        WebcamName backcam = hardwareMap.get(WebcamName.class, "Backcam");

        WebcamName frontcam = hardwareMap.get(WebcamName.class, "frontCam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(backcam, cameraMonitorViewId);

        frontWeb = OpenCvCameraFactory.getInstance().createWebcam(frontcam, cameraMonitorViewId);

        BackWeb.setPipeline(Pole);

        frontWeb.setPipeline(Cone);


        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                BackWeb.getGainControl().setGain(100);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                BackWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    BackWeb.getFocusControl().setFocusLength(450);
                }

                BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) { }
        });

        BackWeb.setPipeline(Pole);

        frontWeb.setPipeline(Cone);

        telemetry.addData("H:", Pole.getH());
        telemetry.addData("S:", Pole.getS());
        telemetry.addData("V:", Pole.getV());
        telemetry.addData("Status:", "Initialized");
        telemetry.addData("top pivot:", Top_Pivot.getPosition());
        telemetry.update();
    }

    public void Pods_Down(){

        Odo_raise.setTargetPosition(0);

        Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Odo_raise.setPower(0.5);

        while (Odo_raise.isBusy()){}

        Odo_raise.setPower(0);


    }

    public void Pods_Up(){

        Odo_raise.setTargetPosition(105);

        Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Odo_raise.setPower(-0.5);

        while (Odo_raise.isBusy()){}

        Odo_raise.setPower(0);
    }

}