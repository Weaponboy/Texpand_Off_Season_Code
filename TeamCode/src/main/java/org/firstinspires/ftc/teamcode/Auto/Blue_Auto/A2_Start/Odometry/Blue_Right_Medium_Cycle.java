package org.firstinspires.ftc.teamcode.Auto.Blue_Auto.A2_Start.Odometry;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeP;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.wolfpackmachina.bettersensors.HardwareMapProvider;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


@Autonomous
public class Blue_Right_Medium_Cycle extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

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

    private MotorEx LF, RF, LB, RB;

    Setpoints setpoints = new Setpoints();

    private ElapsedTime runtime = new ElapsedTime();

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

    private boolean time = false;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.32;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;

    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    public OpenCvWebcam Texpandcamera;


    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap, 0);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        slide.init(hardwareMap, 1);

        OdometryInit();

        InitCamera();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }


        if(tagOfInterest.id == RIGHT){

            runtime.reset();

            Texpandcamera.closeCameraDevice();

            telemetry.addData("Stop Position", "3");
            telemetry.update();

            drive.WithOutEncoders();

            ExtendHighPreloaded();

            //Drop Off Position
            Odo_Drive(124, 0, 48, 0.1, 1, 0);

            top.Top_Pivot.setPosition(0.19);

            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            DropPreLoad();

            CheckVSlidePosForZero();

            Destack_5();

            Pos_3();

        }else if (tagOfInterest.id == LEFT){

            runtime.reset();

            Texpandcamera.closeCameraDevice();

            telemetry.addData("Stop Position", "1");
            telemetry.update();

            drive.WithOutEncoders();

            ExtendHighPreloaded();

            //Drop Off Position
            Odo_Drive(124, 0, 48, 0.1, 1, 0);

            top.Top_Pivot.setPosition(0.19);

            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            DropPreLoad();

            CheckVSlidePosForZero();

            Destack_5();

            Pos_1();

        }else if (tagOfInterest.id == MIDDLE){

            runtime.reset();

            Texpandcamera.closeCameraDevice();

            telemetry.addData("Stop Position", "2");
            telemetry.update();

            drive.WithOutEncoders();

            ExtendHighPreloaded();

            //Drop Off Position
            Odo_Drive(124, 0, 48, 0.1, 1, 0);

            top.Top_Pivot.setPosition(0.19);

            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            DropPreLoad();

            CheckVSlidePosForZero();

            Destack_5();

            Pos_2();

        }

//
//        while (opModeIsActive()){
//
//            odometry.updatePose();
//
//            StartingHeading = Math.toDegrees(getheading());
//
//            if (StartingHeading <= 0) {
//                ConvertedHeading = (360 + StartingHeading);
//            } else {
//                ConvertedHeading = (0 + StartingHeading);
//            }
//
//            double targetRot = 45;
//
//            rotdist = (targetRot - ConvertedHeading);
//
//            rotdistForStop = (targetRot - ConvertedHeading);
//
//            if (rotdist < -180) {
//                rotdist = (360 + rotdist);
//            }else if (rotdist > 180) {
//                rotdist = (rotdist - 360);
//            }
//
//            if (rotdistForStop < -180) {
//                rotdistForStop = (360 + rotdistForStop);
//            } else if (rotdistForStop > 180) {
//                rotdistForStop = (rotdistForStop - 360);
//            }
//
//            rotdist = rotdist*1.45;
//
//            telemetry.addData("heading for stop", rotdistForStop);
//            telemetry.addData("heading With F", rotdist);
//            telemetry.addData("heading", ConvertedHeading);
//            telemetry.addData("X", getXpos());
//            telemetry.addData("Y", getYpos());
//            telemetry.update();
//        }

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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

    public void ExtendMedium (){
        top.Top_Gripper.setPosition(0);

        top.Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        slide.Right_Slide.setTargetPosition(900);
        slide.Left_Slide.setTargetPosition(900);

        top.Top_Pivot.setPosition(0.19);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.Right_Slide.setPower(1);
        slide.Left_Slide.setPower(1);

    }

    public void ExtendHighPreloaded(){
        top.Top_Gripper.setPosition(0);

        top.Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        slide.Right_Slide.setTargetPosition(900);
        slide.Left_Slide.setTargetPosition(900);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.Right_Slide.setPower(1);
        slide.Left_Slide.setPower(1);

    }

    public void DropPreLoad(){

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (slide.Right_Slide.getCurrentPosition() < 800){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        try {
            Thread.sleep(200);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        top.Top_Pivot.setPosition(0);

        try {
            Thread.sleep(300);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        top.Top_Gripper.setPosition(0.3);

        //TO DO: Insert WHILE loop
        if (top.Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(50);
            } catch (Exception e) {
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

    public void DropPreLoadNotSame(){
        top.Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        slide.Right_Slide.setTargetPosition(1800);
        slide.Left_Slide.setTargetPosition(1800);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slide.Right_Slide.getCurrentPosition() < 1750 && slide.Left_Slide.getCurrentPosition() < 1750) {
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);
            top.Top_Pivot.setPosition(0.42);
        }
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);
        top.Top_Pivot.setPosition(0);


        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(400);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        top.Top_Gripper.setPosition(0.3);

        //TO DO: Insert WHILE loop
        if (top.Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(50);
            } catch (Exception e) {
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

    public void CheckVSlidePosForZero(){
        if (slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()) {
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;

        }else if (lowering) {
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }
    }

    public void CheckVSlidePosForDropHigh(){
        if (slide.Right_Slide.getCurrentPosition() < 1750 && slide.Left_Slide.getCurrentPosition() > 1750 ) {

            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);

            Extending_High = true;

        }else{
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Extending_High = false;
        }
    }

    public void CollectCone(double De_pos){

        bottom.Base_Gripper.setPosition(0.4);

        bottom.Destacker_Left.setPosition(De_pos);
        bottom.Destacker_Right.setPosition(De_pos);

        if(bottom.Destacker_Left.getPosition() == setpoints.De_Pos_1){
            bottom.Base_Pivot.setPosition(0.05);
        }else{
            bottom.Base_Pivot.setPosition(0.05);
        }

        top.Top_Pivot.setPosition(0.5);

        slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.Extend.setPower(-1);

        conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 60;

        //extend till we find a cone or get to the slides limit
        while (!conefound && slide.Extend.getCurrentPosition() > -900) {

            CheckVSlidePosForZero();

            conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 60;

            boolean SlowPoint = slide.Extend.getCurrentPosition() < -520;

            if (SlowPoint){
                slide.Extend.setPower(-0.35);
            }else {
                slide.Extend.setPower(-1);
            }

        }
        slide.Extend.setPower(0);

        if (conefound || slide.Extend.getCurrentPosition() <= -890){

            //close gripper
            bottom.Base_Gripper.setPosition(0);

            CheckVSlidePosForZero();

            //make sure gripper is closed
            try {
                Thread.sleep(150);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            bottom.Base_Pivot.setPosition(0.82);

            try {
                Thread.sleep(250);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }


                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (slide.Extend.isBusy()) {

                    CheckVSlidePosForZero();

                    bottom.Base_Pivot.setPosition(0.82);

                    slide.Extend.setPower(0.9);

                    if(slide.Extend.getCurrentPosition() > -300){
                        bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
                        bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);
                    }
                    if(slide.Extend.getCurrentPosition() > -100){
                        bottom.Base_Gripper.setPosition(0.4);
                    }
                    if(slide.Extend.getCurrentPosition() > -75){
                        //open top gripper
                        top.Top_Gripper.setPosition(0.4);

                        //take top pivot to pick up the cone
                        top.Top_Pivot.setPosition(1);
                    }
                }

                slide.Extend.setPower(0);


                while (lowering) {
                    CheckVSlidePosForZero();
                }

                //open base gripper
                bottom.Base_Gripper.setPosition(0.4);

                Nest_Occupied = slide.colour.blue() > 1500;

                bottom.Base_Pivot.setPosition(1);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }


                Nest_Occupied = slide.colour.blue() > 1500;

                if(!Nest_Occupied){
                    try {
                        Thread.sleep(250);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
                }else{
                    try {
                        Thread.sleep(10);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
                }

                Nest_Occupied = slide.colour.blue() > 1500;

                if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
//                    Top_Pivot.setPosition(1);
                }

                if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
//                    Top_Pivot.setPosition(1);
                }

            if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
//                    Top_Pivot.setPosition(1);
            }

            if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
//                    Top_Pivot.setPosition(1);
            }

            if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
//                    Top_Pivot.setPosition(1);
            }

            if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
//                    Top_Pivot.setPosition(1);
            }

            Nest_Occupied = slide.colour.blue() > 1500;


            if (Nest_Occupied) {

                //close top gripper
                top.Top_Gripper.setPosition(0);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

            }else{
                abort = true;
            }

        }else{

            bottom.Base_Pivot.setPosition(0.82);

            slide.Extend.setTargetPosition(0);

            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (slide.Extend.isBusy()) {

                CheckVSlidePosForZero();
                slide.Extend.setPower(0.8);

            }
            slide.Extend.setPower(0);

            bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);

            abort = true;
        }
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot, double error, double Power_For_Long_Drive, double RampPower) {

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
            Xdist = (targetX - CurrentXPos) * 1.4;
            Ydist = (targetY - CurrentYPos) * 1.4;

            XdistForStop = (targetX - CurrentXPos);
            YdistForStop = (targetY - CurrentYPos);

            //CONVERT HEADING FOR TRIG CALCS
            if (StartingHeading <= 0) {
                ConvertedHeading = (360 + StartingHeading);
            } else {
                ConvertedHeading = (0 + StartingHeading);
            }

            rotdist = (targetRot - ConvertedHeading);

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

            rotdist = rotdist*1.45;

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            RRXdist = Xdist * Math.cos(Math.toRadians(360 - ConvertedHeading)) - Ydist * Math.sin(Math.toRadians(360 - ConvertedHeading));

            RRYdist = Xdist * Math.sin(Math.toRadians(360 - ConvertedHeading)) + Ydist * Math.cos(Math.toRadians(360 - ConvertedHeading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = drivePID.calculate(-RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            //SET MOTOR POWER USING THE PID OUTPUT
            drive.RF.setPower(Power_For_Long_Drive*(-Pivot + (Vertical + Horizontal)));
            drive.RB.setPower(Power_For_Long_Drive*((-Pivot * 1.4) + (Vertical - (Horizontal * 1.3))));
            drive.LF.setPower(Power_For_Long_Drive*(Pivot + (Vertical - Horizontal)));
            drive.LB.setPower(Power_For_Long_Drive*((Pivot * 1.4) + (Vertical + (Horizontal * 1.3))));

            telemetry.addData("heading", ConvertedHeading);
            telemetry.addData("Target heading", rotdistForStop);
            telemetry.addData("Pivot Power", Pivot);
            telemetry.addData("X", getXpos());
            telemetry.addData("Y", getYpos());
            telemetry.update();

        }while ((Math.abs(XdistForStop) > 1 + error) || (Math.abs(YdistForStop) > 1 + error) || (Math.abs(rotdistForStop) > 1.2 + error));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void OdometryInit(){

        HardwareMapProvider.setMap(this);

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

        gyro = new Gyro("imu", 0);

        driveTrain = new MecanumDrive(RB, RF, LB, LF);

        leftOdometer = LF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = RF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = RB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, -8, new Rotation2d(0)));

//        odometry.updatePose();

    }

    public void InitCamera(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        Texpandcamera.setPipeline(aprilTagDetectionPipeline);

        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(100);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                Texpandcamera.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    Texpandcamera.getFocusControl().setFocusLength(400);
                }

                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) { }
        });

        Texpandcamera.setPipeline(aprilTagDetectionPipeline);

        telemetry.setMsTransmissionInterval(50);

    }

    public void Pos_1(){
        Odo_Drive(116, -53, 0 , 0.1, 1, 0);
    }

    public void Pos_2(){
        Odo_Drive(116, 0, 0 , 0.1, 1, 0);
    }

    public void Pos_3(){
        Odo_Drive(116, 60, 0 , 0.1, 1, 0);
    }

    public void Destack_4 () {
//
//        //Collect Cone Position
//        Odo_Drive(130, 0, 45, 0);
//
//        //Collect Cone Position
//        Odo_Drive(130, 20, 90, 0);
//
//        bottom.Base_Gripper.setPosition(0.4);
//
//        bottom.Base_Pivot.setPosition(0.12);
//
//        try {
//            Thread.sleep(350);
//        } catch (Exception e) {
//            System.out.println(e.getMessage());
//        }
//
//        //cone 1
//        CollectCone(setpoints.De_Pos_1);
//
//        //Drop Off Position
//        Odo_Drive(124, 11, 131 , 0);
//
//        if (!abort){
//            DropPreLoadNotSame();
//        }
//
//        if (abort){
//
//            //Drive to position
//            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
//
//            bottom.Base_Pivot.setPosition(0.72);
//
//        }else {
////
////            //Collect Cone Position
////            Odo_Drive(130, 0, 90);
//
//            //Collect Cone Position
//            Odo_Drive(130, 20, 90, 0);
//
//            bottom.Base_Gripper.setPosition(0.4);
//
//            bottom.Base_Pivot.setPosition(0.12);
//
//            try {
//                Thread.sleep(350);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//
//            //cone 1
//            CollectCone(setpoints.De_Pos_2);
//
//            //Drop Off Position
//            Odo_Drive(124, 11, 131, 0);
//
//            if (!abort){
//                DropPreLoadNotSame();
//            }
//
//            if (abort){
//
//                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
//
//                bottom.Base_Pivot.setPosition(0.72);
//
//            }else {
//
////                //Collect Cone Position
////                Odo_Drive(130, 0, 90);
//
//                //Collect Cone Position
//                Odo_Drive(130, 20, 90, 0);
//
//                bottom.Base_Gripper.setPosition(0.4);
//
//                bottom.Base_Pivot.setPosition(0.12);
//
//                try {
//                    Thread.sleep(350);
//                } catch (Exception e) {
//                    System.out.println(e.getMessage());
//                }
//
//                //cone 1
//                CollectCone(setpoints.De_Pos_3);
//
//                //Drop Off Position
//                Odo_Drive(124, 11, 131, 0);
//
//                if (!abort){
//                    DropPreLoadNotSame();
//                }
//
//                if (abort){
//
//                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
//
//                    bottom.Base_Pivot.setPosition(0.72);
//
//                }else {
//
////                    //Collect Cone Position
////                    Odo_Drive(130, 0, 90);
//
//                    //Collect Cone Position
//                    Odo_Drive(130, 20, 90, 0);
//
//                    bottom.Base_Gripper.setPosition(0.4);
//
//                    bottom.Base_Pivot.setPosition(0.12);
//
//                    try {
//                        Thread.sleep(350);
//                    } catch (Exception e) {
//                        System.out.println(e.getMessage());
//                    }
//
//                    //cone 1
//                    CollectCone(setpoints.De_Pos_4);
//
//                    //Drop Off Position
//                    Odo_Drive(124, 11, 131, 0);
//
//                    if (!abort){
//                        DropPreLoadNotSame();
//                    }
//                }
//            }
//
//        }

    }

    public void Destack_5 () {

        bottom.Base_Gripper.setPosition(0.4);

        bottom.Base_Pivot.setPosition(0.05);

        //Collect Cone Position
        Odo_Drive(120, 20, 90, 0, 1, 0);

        //cone 1
        CollectCone(setpoints.De_Pos_1);

        top.Top_Gripper.setPosition(0);

        time  = runtime.milliseconds() > 27000;

        if (!abort && !time){
            top.Top_Gripper.setPosition(0);

            ExtendMedium();

            //Drop Off Position
            Odo_Drive(120, 0, 46, 0.1, 1, 0);

            DropPreLoad();
        }

        time = runtime.milliseconds() > 27000;

        if (abort || time){

            //Drive to position
            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

            bottom.Base_Pivot.setPosition(0.72);

        }else {

            bottom.Base_Pivot.setPosition(0.2);

            //Collect Cone Position
            Odo_Drive(120, 20, 90, 0, 1, 0.1);

            bottom.Base_Gripper.setPosition(0.4);

            bottom.Base_Pivot.setPosition(0.05);

            //cone 2
            CollectCone(setpoints.De_Pos_2);

            top.Top_Gripper.setPosition(0);

            time = runtime.milliseconds() > 27000;

            //Drop Off Position
            if (!abort && !time){

                top.Top_Gripper.setPosition(0);

                ExtendMedium();

                //Drop Off Position
                Odo_Drive(120, 0, 46, 0.1, 1, 0);

                DropPreLoad();
            }

            time = runtime.milliseconds() > 27000;

            if (abort || time){

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                bottom.Base_Pivot.setPosition(0.72);
            } else {

                bottom.Base_Pivot.setPosition(0.2);

                //Collect Cone Position
                Odo_Drive(120, 20, 90, 0, 1, 0.1);

                bottom.Base_Gripper.setPosition(0.4);

                bottom.Base_Pivot.setPosition(0.05);

                //cone 3
                CollectCone(setpoints.De_Pos_3);

                top.Top_Gripper.setPosition(0);

                time = runtime.milliseconds() > 27000;

                if (!abort && !time){

                    top.Top_Gripper.setPosition(0);

                    ExtendMedium();

                    //Drop Off Position
                    Odo_Drive(120, 0, 46, 0.1, 1, 0);

                    DropPreLoad();
                }

                time = runtime.milliseconds() > 27000;

                if (abort || time){

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                    bottom.Base_Pivot.setPosition(0.72);

                }else {

                    bottom.Base_Pivot.setPosition(0.2);

                    //Collect Cone Position
                    Odo_Drive(120, 20, 90, 0, 1, 0.1);

                    bottom.Base_Gripper.setPosition(0.4);

                    bottom.Base_Pivot.setPosition(0.05);

                    //cone 4
                    CollectCone(setpoints.De_Pos_4);

                    top.Top_Gripper.setPosition(0);

                    abort = runtime.milliseconds() > 27000;

                    if (!abort && !time){

                        top.Top_Gripper.setPosition(0);

                        ExtendMedium();

                        //Drop Off Position
                        Odo_Drive(120, 0, 46, 0.1, 1, 0);

                        DropPreLoad();
                    }

                    time = runtime.milliseconds() > 27000;

                    if (abort || time) {

                        top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                        bottom.Base_Pivot.setPosition(0.72);

                    }else {

                        bottom.Base_Pivot.setPosition(0.2);

                        //Collect Cone Position
                        Odo_Drive(120, 20, 90, 0, 1, 0.1);

                        bottom.Base_Gripper.setPosition(0.4);

                        bottom.Base_Pivot.setPosition(0.05);

                        //cone 5
                        CollectCone(setpoints.De_Pos_5);

                        top.Top_Gripper.setPosition(0);

                        time = runtime.milliseconds() > 27000;

                        if (!abort && !time){

                            top.Top_Gripper.setPosition(0);

                            ExtendMedium();

                            //Drop Off Position
                            Odo_Drive(120, 0, 46, 0.1, 1, 0);

                            DropPreLoad();
                        }

                    }
                }
            }

        }

    }

    public void Destack_3 () {
//
//        //Collect Cone Position
//        Odo_Drive(130, 0, 90, 0);
//
//        //Collect Cone Position
//        Odo_Drive(130, 20, 90, 0);
//
//        bottom.Base_Gripper.setPosition(0.4);
//
//        bottom.Base_Pivot.setPosition(0.12);
//
//        try {
//            Thread.sleep(350);
//        } catch (Exception e) {
//            System.out.println(e.getMessage());
//        }
//
//        //cone 1
//        CollectCone(setpoints.De_Pos_1);
//
//        //Drop Off Position
//        Odo_Drive(112, 0, 148, 0);
//
//        if (!abort){
//            DropPreLoadNotSame();
//        }
//
//        if (abort){
//
//            //Drive to position
//            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
//
//            bottom.Base_Pivot.setPosition(0.72);
//
//        }else {
//
//            //Collect Cone Position
//            Odo_Drive(130, 0, 90, 0);
//
//            //Collect Cone Position
//            Odo_Drive(130, 20, 90, 0);
//
//            bottom.Base_Gripper.setPosition(0.4);
//
//            bottom.Base_Pivot.setPosition(0.12);
//
//            try {
//                Thread.sleep(350);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//
//            //cone 1
//            CollectCone(setpoints.De_Pos_2);
//
//            //Drop Off Position
//            Odo_Drive(112, 0, 148, 0);
//
//            if (!abort){
//                DropPreLoadNotSame();
//            }
//
//            if (abort){
//
//                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
//
//                bottom.Base_Pivot.setPosition(0.72);
//
//            }else {
//
//                //Collect Cone Position
//                Odo_Drive(130, 0, 90, 0);
//
//                //Collect Cone Position
//                Odo_Drive(130, 20, 90, 0);
//
//                bottom.Base_Gripper.setPosition(0.3);
//
//                bottom.Base_Pivot.setPosition(0.12);
//
//                try {
//                    Thread.sleep(350);
//                } catch (Exception e) {
//                    System.out.println(e.getMessage());
//                }
//
//                //cone 1
//                CollectCone(setpoints.De_Pos_3);
//
//                //Drop Off Position
//                Odo_Drive(112, 0, 148, 0);
//
//                if (!abort){
//                    DropPreLoadNotSame();
//                }
//            }
//
//        }

    }

}

