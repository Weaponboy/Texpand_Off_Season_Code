package org.firstinspires.ftc.teamcode.Teleop.Test_Class;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp

public class Blue_Driver_Gamepad_Test extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvWebcam FrontWeb;
    OpenCvWebcam BackWeb;
    public double Degrees_To_Turn;
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;
    public Orientation yawAngle;
    public BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    public static final double CENTER_WHEEL_OFFSET = -17;

    public static final double WHEEL_DIAMETER = 3.5;

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private boolean lowering = false;

    private boolean Nest_Occupied = false;

    private boolean Extending_High = false;

    private MecanumDrive driveTrain;

    private boolean conefound = false;

    private boolean abort = false;

    private MotorEx LEFTF, RIGHTF, LEFTB, RIGHTB;

    Setpoints setpoints = new Setpoints();

    Gyro gyro;

    PIDFController drivePID;
    PIDFController strafePID;
    PIDFController PivotPID;

    double Xdist = 0;
    double Ydist = 0;

    double rotdist = 0;

    double XdistForStop = 0;
    double YdistForStop = 0;

    double rotdistForStop = 0;

    double RRXdist = 0;
    double RRYdist = 0;
    double Horizontal = 0;
    double Vertical = 0;

    double Horizontal2 = 0;
    double Vertical2 = 0;

    double ConvertedHeading = 0;
    double Pivot = 0;

    Top_gripper top = new Top_gripper();

    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();

    Slides slide = new Slides();

    double CurrentXPos = 0;
    double CurrentYPos = 0;

    double StartingHeading = 0;

    double StartingHeadinggyro = 0;

    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.32;

    public boolean conefoundcycle;

    private double power;

    public DcMotor Odo_raise  = null;

    Drivetrain drive = new Drivetrain();

    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();

    Pole_Pipe Pole;

    double De_Pos_1 = 0.0;
    double De_Pos_2 = 0.12;
    double De_Pos_3 = 0.45;
    double De_Pos_4 = 0.6;
    double De_Pos_5 = 0.72;

    public ColorSensor colour = null;

    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;

    public DcMotor Extend = null;

    public DigitalChannel SlideZero;

    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;
    public Servo Top_Gripper = null;
    public Servo Top_Pivot = null;
    public Servo Destacker_Left = null;
    public Servo Destacker_Right = null;

    private DistanceSensor sensorRange;

    private DistanceSensor Back_Distance;

    private double vertical;
    private double horizontal;
    private double pivot;

    private double Base_Pivot_Collect = 0.05;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;
    Blue_Cone_Pipe Cone_Pipeline;
    private double Top_Pivot_Collect = 0.35;

    private double Top_Gripper_Collect_Wide = 0.36;

    private boolean rumble = false;

    private int Toppos = 0;

    private int Toppos2 = 0;

    private int stakerpos = 0;
    private double Destack_position = 0;


    private  double poleDistance = 0;

    private  double TargetPoleDistance = 420;

    private  double TravelDistance = 0;

    private  int Loop = 0;
    private  int Loop2 = 0;

    public double Distance_To_Travel;

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();


    public double ConversionPixelstoCm = 22;//need to tune this

    public double CenterOfScreen = 300;
    private int TopposP = 0;

    private boolean PoleAlignmnet = true;

    public double rectPositionFromLeft = 0;

    private boolean SlowPoint = false;

    private double slow = 0.4;
    private double Cone_power;
    private double slow1 = 0.4;

    public double time = 0;


    @Override
    public void loop() {

        if (time == 0){
            time++;
            runtime.reset();
        }

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        slow1 = (gamepad1.left_trigger * 0.6) + 0.4;

        drive.WithOutEncoders();

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower((slow1*1.4)*(-pivot + (vertical - horizontal)));
        RB.setPower(slow1*(-pivot + (vertical + horizontal)));
        LF.setPower((slow1*1.4)*(pivot + (vertical + horizontal)));
        LB.setPower(slow1*(pivot + (vertical - horizontal)));

        if (gamepad2.left_stick_button && gamepad1.left_stick_button){

            TopposP = TopposP + 1;

            if(TopposP == 1){
                PoleAlignmnet = false;
            }else if(TopposP == 3){
                PoleAlignmnet = true;
            }

            if(TopposP > 2){
                Toppos = 0;
            }

        }

        if(!currentGamepad2.b && previousGamepad2.b && Base_Gripper.getPosition() == 0) {
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }else if(!currentGamepad2.b && previousGamepad2.b && Base_Gripper.getPosition() > 0){
            Base_Gripper.setPosition(0); //close base gripper if it is open
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (gamepad2.dpad_up) {
            AlignToLowCalc();

            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            AlignToPoleNoEncoders();
        }

        if (gamepad1.dpad_up) {
            Destacker_Left.setPosition(De_Pos_1);
            Destacker_Right.setPosition(De_Pos_1);
            Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.right_bumper) {
            Destacker_Left.setPosition(De_Pos_2);
            Destacker_Right.setPosition(De_Pos_2);
            Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.y) {
            Destacker_Left.setPosition(De_Pos_3);
            Destacker_Right.setPosition(De_Pos_3);
            Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.b) {
            Destacker_Left.setPosition(De_Pos_4);
            Destacker_Right.setPosition(De_Pos_4);
            Base_Pivot.setPosition(Base_Pivot_Collect);
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

        if(gamepad2.back){

            Base_Pivot.setPosition(Base_Pivot_Collect);

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

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

                SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 200;

                if (SlowPoint){
                    Extend.setPower(-0.5);
                }else {
                    Extend.setPower(-1);
                }


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

                Top_Pivot.setPosition(0.6);

                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                Base_Pivot.setPosition(Base_Pivot_Flip);

                if (Destacker_Left.getPosition() < 0.6){

                    try {
                        Thread.sleep(400);
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

                        if(Extend.getCurrentPosition() > -300){
                            Destacker_Left.setPosition(De_Pos_5);
                            Destacker_Right.setPosition(De_Pos_5);
                        }

                        Base_Gripper.setPosition(0);

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Extend.setPower(0.8);


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
        }

        if (gamepad1.start){
            Odo_Drive(180, 0, 1);
        }

        if (runtime.milliseconds() > 90000){
            gamepad2.runRumbleEffect(customRumbleEffect);
        }

        if(gamepad2.start){

            Base_Pivot.setPosition(0.7);
            try {
                Thread.sleep(600);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Cone_power = 0.2;
            rectPositionFromLeft = Cone_Pipeline.getRectX();
            while (Math.abs(rectPositionFromLeft - CenterOfScreen) > 25 && (Math.abs(gamepad1.right_stick_x) < 0.1)){

                if(Math.abs(rectPositionFromLeft - CenterOfScreen) > 15 ){
                    Cone_power = 0.15;
                }
                rectPositionFromLeft = Cone_Pipeline.getRectX();

                if (rectPositionFromLeft < CenterOfScreen + 10) {
                    drive.RF.setPower(1.3*Cone_power);
                    drive.RB.setPower(Cone_power);
                    drive.LF.setPower(-1.3*Cone_power);
                    drive.LB.setPower(-Cone_power);
                } else if (rectPositionFromLeft > CenterOfScreen - 10) {
                    drive.RF.setPower(-1.3*Cone_power);
                    drive.RB.setPower(-Cone_power);
                    drive.LF.setPower(1.3*Cone_power);
                    drive.LB.setPower(Cone_power);
                }

            }
            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            Base_Pivot.setPosition(0.05);

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


        //toggle position of top pivot
        if (gamepad1.x) {
            Top_Pivot.setPosition(0.3);
        }

        if (gamepad2.x) {
            Top_Pivot.setPosition(1);
        }

        //set top pivot postition
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Top_Pivot.setPosition(0);
        }

        //toggle positioin of top gripper
        if(!currentGamepad2.y && previousGamepad2.y && Top_Gripper.getPosition() == 0){
            Top_Gripper.setPosition(Top_Pivot_Collect); //lift up top griper if it is down
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }else if(!currentGamepad2.y && previousGamepad2.y && Top_Gripper.getPosition() > 0){
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

                    SlowPoint = sensorRange.getDistance(DistanceUnit.MM) < 180;

                    if (SlowPoint){
                        Extend.setPower(-0.55);
                    }else {
                        Extend.setPower(-1);
                    }

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

                    Top_Pivot.setPosition(0.6);

                    Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                    try {
                        Thread.sleep(125);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

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

                            Base_Pivot.setPosition(0.82);

                            Base_Gripper.setPosition(0);

                            Extend.setPower(0.6);

//                            if(Extend.getCurrentPosition() > -90){
//                                Base_Gripper.setPosition(0.4);
//                            }
                            if(Extend.getCurrentPosition() > -40){
                                //open top gripper
                                Top_Gripper.setPosition(0.35);

                                //take top pivot to pick up the cone
                                Top_Pivot.setPosition(1);
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
                                Thread.sleep(75);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Base_Pivot.setPosition(Base_Pivot_Out_Way);

                            try {
                                Thread.sleep(50);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(1);

                            try {
                                Thread.sleep(50);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Gripper.setPosition(0);

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(0.6);

                            try {
                                Thread.sleep(50);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }
                            Base_Pivot.setPosition(Base_Pivot_Collect);
                        }


                    Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Right_Slide.setTargetPosition(1850);
                    Left_Slide.setTargetPosition(1850);
                    Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Right_Slide.getCurrentPosition() < 1800 && Left_Slide.getCurrentPosition() < 1800) {
                        Right_Slide.setPower(1);
                        Left_Slide.setPower(1);

                        Top_Pivot.setPosition(0.15);

                        if(Right_Slide.getCurrentPosition() > 1800){
                            Top_Pivot.setPosition(0);
                        }
                    }
                    Right_Slide.setPower(0.005);
                    Left_Slide.setPower(0.005);

                    Top_Pivot.setPosition(0.1);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Gripper.setPosition(Top_Pivot_Collect);

                    if(Top_Gripper.getPosition() == Top_Pivot_Collect) {

                        Right_Slide.setTargetPosition(0);
                        Left_Slide.setTargetPosition(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        Top_Pivot.setPosition(0.4);

                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);

                        lowering = true;
                    }

                }else{
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Extend.setPower(0.8);

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

                    Base_Pivot.setPosition(Base_Pivot_Collect);
                }



        }

        //Align to the top pole
        if(gamepad2.dpad_left || gamepad1.dpad_left){

            AlignToHighCalc();

            Right_Slide.setTargetPosition(1850);
            Left_Slide.setTargetPosition(1850);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            drive.DriveEncoders();

            power = 0.2;

            while(Right_Slide.getCurrentPosition() < 1800 && Left_Slide.getCurrentPosition() < 1800){

                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();

                if (PoleAlignmnet){
                    AlignToPoleNoEncoders();
                }

                Top_Pivot.setPosition(0.19);

            }

            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);
            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rectPositionFromLeft = Pole.getRectX();

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            Top_Pivot.setPosition(0.19);

        }

        if(gamepad1.left_stick_y > 0.5 && Base_Pivot.getPosition() != 0.85){
            Base_Pivot.setPosition(0.85); //open base gripper if it is closed
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }else if(gamepad1.left_stick_y < -0.5 && Base_Pivot.getPosition() != Base_Pivot_Collect ){
            Base_Pivot.setPosition(Base_Pivot_Collect); //close base gripper if it is open
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if(gamepad2.left_stick_y < 0 && Base_Pivot.getPosition() != 0.85){
            Base_Pivot.setPosition(0.85); //open base gripper if it is closed
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }else if(gamepad2.left_stick_y > 0 && Base_Pivot.getPosition() != Base_Pivot_Collect ){
            Base_Pivot.setPosition(Base_Pivot_Collect); //close base gripper if it is open
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right || gamepad2.dpad_right){

            Loop2 = 0;
            Loop = 0;

            AlignToMedCalc();

            Right_Slide.setTargetPosition(900);
            Left_Slide.setTargetPosition(900);

            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
//            drive.DriveEncoders();

            while(Right_Slide.getCurrentPosition() < 850 && Left_Slide.getCurrentPosition() < 850){

                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();

                if (PoleAlignmnet){
                    AlignToPoleNoEncoders();
                }

                Top_Pivot.setPosition(0.19);

            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rectPositionFromLeft = Pole.getRectX();

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            Top_Pivot.setPosition(0.19);

        }

        //Stop slides if finished running
        if(lowering) {
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        if (gamepad2.right_bumper){

            while (SlideZero.getState() == true){
                Right_Slide.setPower(-0.2);
                Left_Slide.setPower(-0.2);
            }

            telemetry.addData("Loop", "entered");
            telemetry.update();

            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        //extend slides to collect cone
        if (gamepad2.left_trigger > 0 || gamepad1.back) {
//
//            if(Destacker_Right.getPosition() < 0.5){
//                Base_Pivot.setPosition(0.1);
//            }else{
//
//            }

            Base_Pivot.setPosition(Base_Pivot_Collect);

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);
            conefound = sensorRange.getDistance(DistanceUnit.MM) < 75;

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

                conefound = sensorRange.getDistance(DistanceUnit.MM) < 75;

                if(Destacker_Right.getPosition() < 0.5){
                    SlowPoint = sensorRange.getDistance(DistanceUnit.MM) < 250;
                }else{
                    SlowPoint = sensorRange.getDistance(DistanceUnit.MM) < 180;
                }


                if (SlowPoint && Destacker_Right.getPosition() > 0.5){
                    Extend.setPower(-0.5);
                }else if(SlowPoint && Destacker_Right.getPosition() < 0.5){
                    Extend.setPower(-0.3);
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

                if (Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()) {
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                } else if (lowering) {
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }

                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                try {
                    Thread.sleep(125);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Base_Pivot.setPosition(Base_Pivot_Flip);

                if (Destacker_Left.getPosition() < 0.6) {

                    try {
                        Thread.sleep(400);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                }

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (Extend.isBusy()) {
                    if (Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()) {
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    } else if (lowering) {
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }

                    Base_Pivot.setPosition(Base_Pivot_Flip);

                    Extend.setPower(1);

                    if(Extend.getCurrentPosition() > -300){
                        Destacker_Left.setPosition(De_Pos_5);
                        Destacker_Right.setPosition(De_Pos_5);
                    }

                    if(Extend.getCurrentPosition() > -50){
                        //open top gripper
                        Top_Gripper.setPosition(0.35);

                        //take top pivot to pick up the cone
                        Top_Pivot.setPosition(1);
                    }

                }

                if (Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()) {
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                } else if (lowering) {
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }

                Extend.setPower(0);

                Destacker_Left.setPosition(De_Pos_5);
                Destacker_Right.setPosition(De_Pos_5);

                if (Base_Pivot.getPosition() > 0.6) {

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Base_Gripper.setPosition(0.4);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }


                    Base_Pivot.setPosition(Base_Pivot_Out_Way);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Pivot.setPosition(1);

                    try {
                        Thread.sleep(400);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(200);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Pivot.setPosition(0.4);

                    try {
                        Thread.sleep(75);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    conefoundcycle = colour.blue() > 2000;

                    if (!conefoundcycle) {
                        Base_Pivot.setPosition(Base_Pivot_Collect);
                        try {
                            Thread.sleep(200);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }
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

        FtcDashboard.getInstance().startCameraStream(BackWeb,30);

        odometry.updatePose();

        telemetry.addData("Pole Vision", PoleAlignmnet);

        telemetry.addData("Heading", Math.toDegrees(getheading()));
        telemetry.addData("Y:", getYpos());
        telemetry.addData("X:", getXpos());

        telemetry.addData("Limit", SlideZero.getState());
        telemetry.addData("Destacker Left:", Destacker_Left.getPosition());
        telemetry.addData("Destacker Right:", Destacker_Right.getPosition());

        telemetry.addData("MM range b:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("MM range f", Back_Distance.getDistance(DistanceUnit.MM));
        telemetry.addData("Red:", colour.blue());
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        Odo_raise = hardwareMap.get(DcMotor.class, "Odo_motor");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        SlideZero = hardwareMap.get(DigitalChannel.class, "Limit");

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

        OdometryInit();

        Top_Gripper.setPosition(0.3);

        Base_Gripper.setPosition(0.4);

        drive.init(hardwareMap, 1);

        Back_Distance.resetDeviceConfigurationForOpMode();

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 100)  //  Rumble right motor 100% for 500 mSec
                .build();

        SlideZero.setMode(DigitalChannel.Mode.INPUT);

        Pole = new Pole_Pipe();

        Cone_Pipeline = new Blue_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        FrontWeb  = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"), viewportContainerIds[1]);

        FrontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                FrontWeb.setPipeline(Cone_Pipeline);

                FrontWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                FrontWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);

                FrontWeb.getGainControl().setGain(1);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                FrontWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    FrontWeb.getFocusControl().setFocusLength(450);
                }

                FrontWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        FrontWeb.setPipeline(Cone_Pipeline);

        BackWeb.setPipeline(Pole);
        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {


                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(13, TimeUnit.MILLISECONDS);

                BackWeb.getGainControl().setGain(1);

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
        FtcDashboard.getInstance().startCameraStream(BackWeb,30);
        // Set the pipeline depending on id


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("H:", Pole.getH());
        telemetry.addData("S:", Pole.getS());
        telemetry.addData("V:", Pole.getV());
        telemetry.addData("Status:", "Initialized");
        telemetry.addData("top pivot:", Top_Pivot.getPosition());
        telemetry.update();
    }

    public void AlignToHighCalc() {
        for (int i = 1; i <= 50; i++) {
            rectPositionFromLeft = Pole.TargetHighrectX;
        }

        if (rectPositionFromLeft > -1) {
            Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

            Degrees_To_Turn = Degrees_To_Turn / 20;

            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }

    public void AlignToLowCalc() {
        for (int i = 1; i <= 50; i++) {
            rectPositionFromLeft = Pole.TargetLowrectX;
        }

        if (rectPositionFromLeft > -1) {
            Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

            Degrees_To_Turn = Degrees_To_Turn / 20;

            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }

    public void AlignToMedCalc() {

        for (int i = 1; i <= 50; i++) {
            rectPositionFromLeft = Pole.TargetMedrectX;
        }
        if (rectPositionFromLeft > -1) {
            Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

            Degrees_To_Turn = Degrees_To_Turn / 20;

            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }

    public void AlignToPole(){
        if(Degrees_To_Turn != 0) {
            drive.TurnToHeading(yawAngle.firstAngle + Degrees_To_Turn, 0.3);
            Degrees_To_Turn = 0;
        }
    }

    public void AlignToPoleNoEncoders(){
        if(Degrees_To_Turn != 0) {
            drive.TurnToHeadingNoEncoders(yawAngle.firstAngle + Degrees_To_Turn, 0.6);
            Degrees_To_Turn = 0;
        }
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

    public void OdometryInit(){

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        LEFTF = new MotorEx(hardwareMap, "LF");
        LEFTB = new MotorEx(hardwareMap, "LB");
        RIGHTF = new MotorEx(hardwareMap, "RF");
        RIGHTB = new MotorEx(hardwareMap, "RB");

        driveTrain = new MecanumDrive(RIGHTB, RIGHTF, LEFTB, LEFTF);

        leftOdometer = LEFTF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = RIGHTF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = RIGHTB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(3.141)));

    }

    public double getXpos() {
        return odometry.getPose().getX();
    }

    public double getYpos() {
        return odometry.getPose().getY();
    }

    public double getheading() {
        return odometry.getPose().getHeading();
    }

    public void CheckVSlidePosForZero(){
        if (Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()) {
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;

        }else if (lowering) {
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot, double error, double Power_For_Long_Drive) {

        do {

            CheckVSlidePosForZero();

            //UPDATE ODOMETRY
            odometry.updatePose();

            //GET CURRENT X
            CurrentXPos = getXpos();

            //GET CURRENT Y
            CurrentYPos = getYpos();
//
//            gyro.update();
//
//            //GET START HEADING WITH GYRO
//            StartingHeadinggyro = gyro.angle();

            //GET START HEADING WITH ODOMETRY
            StartingHeading = Math.toDegrees(getheading());

            //PID FOR DRIVING IN THE Y DIRECTION
            drivePID.setPIDF(driveP, 0, driveD, driveF);

            //PID FOR DRIVING IN THE X DIRECTION
            strafePID.setPIDF(strafeP, 0, strafeD, strafeF);

            //PID FOR TURNING
            PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

            //SET DISTANCE TO TRAVEL ERROR
            Xdist = (targetX - CurrentXPos) * 1.3;
            Ydist = (targetY - CurrentYPos) * 1.3;

            XdistForStop = (targetX - CurrentXPos);
            YdistForStop = (targetY - CurrentYPos);

            //CONVERT HEADING FOR TRIG CALCS
            if (StartingHeading <= 0) {
                ConvertedHeading = (360 + StartingHeading);
            } else {
                ConvertedHeading = (0 + StartingHeading);
            }

            rotdist = (targetRot - ConvertedHeading)*1.5;

            rotdistForStop = (targetRot - ConvertedHeading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 180) {
                rotdist = (rotdist - 360);
            }

            if (rotdistForStop < -180) {
                rotdistForStop = (360 + rotdistForStop);
            } else if (rotdistForStop > 180) {
                rotdistForStop = (rotdistForStop - 360);
            }

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            RRXdist = Xdist * Math.cos(Math.toRadians(360 - ConvertedHeading)) - Ydist * Math.sin(Math.toRadians(360 - ConvertedHeading));

            RRYdist = Xdist * Math.sin(Math.toRadians(360 - ConvertedHeading)) + Ydist * Math.cos(Math.toRadians(360 - ConvertedHeading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = drivePID.calculate(-RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            if ((Math.abs(rotdistForStop) < 1.5)){
                Pivot = Pivot*1.5;
            }

            if ((Math.abs(XdistForStop) < 1.5)){
                Vertical = Vertical*1.3;
                Horizontal = Horizontal*1.3;
            }

            if ((Math.abs(YdistForStop) < 1.5)){
                Vertical = Vertical*1.3;
                Horizontal = Horizontal*1.3;
            }

            //SET MOTOR POWER USING THE PID OUTPUT
            drive.RF.setPower(Power_For_Long_Drive*(-Pivot + (Vertical + Horizontal)));
            drive.RB.setPower(Power_For_Long_Drive*((-Pivot * 1.4) + (Vertical - (Horizontal * 1.3))));
            drive.LF.setPower(Power_For_Long_Drive*(Pivot + (Vertical - Horizontal)));
            drive.LB.setPower(Power_For_Long_Drive*((Pivot * 1.4) + (Vertical + (Horizontal * 1.3))));

            telemetry.addData("heading", ConvertedHeading);
            telemetry.addData("Target", rotdistForStop);
            telemetry.addData("Target w F", rotdist);
            telemetry.addData("X", getXpos());
            telemetry.addData("Y", getYpos());
            telemetry.update();

        }while ((Math.abs(XdistForStop) > 0.8 + error) || (Math.abs(YdistForStop) > 0.8 + error) || (Math.abs(rotdistForStop) > 1 + error));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void Odo_Drive(double targetRot, double error, double Power_For_Long_Drive){

        do {

            CheckVSlidePosForZero();

            //UPDATE ODOMETRY
            odometry.updatePose();

            //GET START HEADING WITH ODOMETRY
            StartingHeading = Math.toDegrees(getheading());

            //PID FOR TURNING
            PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

            //CONVERT HEADING FOR TRIG CALCS
            if (StartingHeading <= 0) {
                ConvertedHeading = (360 + StartingHeading);
            } else {
                ConvertedHeading = (0 + StartingHeading);
            }

            rotdist = (targetRot - ConvertedHeading)*1.3;

            rotdistForStop = (targetRot - ConvertedHeading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 180) {
                rotdist = (rotdist - 360);
            }

            if (rotdistForStop < -180) {
                rotdistForStop = (360 + rotdistForStop);
            } else if (rotdistForStop > 180) {
                rotdistForStop = (rotdistForStop - 360);
            }

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            Pivot = PivotPID.calculate(-rotdist);

//            if ((Math.abs(rotdistForStop) < 1.5)){
//                Pivot = Pivot*1.2;
//            }

            //SET MOTOR POWER USING THE PID OUTPUT
            RF.setPower(Power_For_Long_Drive*(-Pivot + (Vertical + Horizontal)));
            RB.setPower(Power_For_Long_Drive*((-Pivot * 1.4) + (Vertical - (Horizontal * 1.3))));
            LF.setPower(Power_For_Long_Drive*(Pivot + (Vertical - Horizontal)));
            LB.setPower(Power_For_Long_Drive*((Pivot * 1.4) + (Vertical + (Horizontal * 1.3))));

            telemetry.addData("heading", ConvertedHeading);
            telemetry.addData("Target", rotdistForStop);
            telemetry.addData("Target w F", rotdist);
            telemetry.addData("X", getXpos());
            telemetry.addData("Y", getYpos());
            telemetry.update();

        }while ((Math.abs(rotdistForStop) > 1 + error));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }
}