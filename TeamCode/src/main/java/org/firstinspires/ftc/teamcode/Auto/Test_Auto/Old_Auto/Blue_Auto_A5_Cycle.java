/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto.Test_Auto.Old_Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous
public class Blue_Auto_A5_Cycle extends LinearOpMode {
    private DistanceSensor sensorRange;

    double Top_Open_Wide = 0.36;
    double Top_Open = 0.33;
    double Top_Closed = 0;

    public DcMotor Odo_raise  = null;

    double Top_Pivot_Collect = 1;
    double Top_Pivot_Waiting = 0.6;
    double Top_Pivot_Ready_Drop = 0.3;
    double Top_Pivot_Drop_Off = 0;

    public ColorSensor colour = null;

    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;

    public DcMotor Extend = null;

    public Servo Base_Gripper;
    public Servo Base_Pivot;
    public Servo Top_Gripper;
    public Servo Top_Pivot;
    public Servo Destacker_Left;
    public Servo Destacker_Right;

    private boolean Nest_Occupied = false;
    private boolean conefound = false;
    private boolean extending = false;
    private double slow = 1;
    private boolean lowering = false;
    private boolean abort = false;

    public static final double TRACKWIDTH = 36.2  ;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static final double CENTER_WHEEL_OFFSET = -13;

    public static final double WHEEL_DIAMETER = 3.5;

    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private double vertical;
    private double horizontal;
    private double pivot;


    private MotorEx LF, RF, LB, RB;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;


    // destacker positions
    double De_Pos_1 = 0.34;
    double De_Pos_2 = 0.44;
    double De_Pos_3 = 0.54;
    double De_Pos_4 = 0.64;
    double De_Pos_5 = 0.84;

    OpenCvCamera Texpandcamera;

    Blue_Cone_Pipe colin;

    public double Distance_To_Travel;

    public double ConversionPixelstoCm = 20;//need to tune this

    public double CenterOfScreen = 320;

    public double rectPositionFromLeft = 0;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Drivetrain drive = new Drivetrain();

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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

    @Override
    public void runOpMode() {
        initialize();

        colin = new Blue_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvWebcam Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        Texpandcamera.setPipeline(aprilTagDetectionPipeline);

        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(100);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                Texpandcamera.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    Texpandcamera.getFocusControl().setFocusLength(500);
                }

                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        drive.init(hardwareMap, 1);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

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

        if (tagOfInterest.id == RIGHT) {
            //Position 3
            telemetry.addData("Stop Position", "3");
            telemetry.update();

            Texpandcamera.setPipeline(colin);


            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_3();

        } else if (tagOfInterest.id == LEFT) {
            //Position 1
            telemetry.addData("Stop Position", "1");
            telemetry.update();

            Texpandcamera.setPipeline(colin);

            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_1();

        } else if (tagOfInterest.id == MIDDLE) {
            //Position 2
            telemetry.addData("Stop Position", "2");
            telemetry.update();

            Texpandcamera.setPipeline(colin);

            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_2();

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
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

    public void initialize() {

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

        //Telemetry for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        odometry.updatePose(new Pose2d(0, 248, new Rotation2d()));

        odometry.update(0, 0, 0);

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Base_Gripper = hardwareMap.get(Servo.class, "Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class, "Base_Pivot");
        Top_Gripper = hardwareMap.get(Servo.class, "Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class, "Top_Pivot");
        Destacker_Left = hardwareMap.get(Servo.class, "Destacker Left");
        Destacker_Right = hardwareMap.get(Servo.class, "Destacker Right");

        Odo_raise = hardwareMap.get(DcMotor.class, "Odo_motor");

        Destacker_Left.setDirection(Servo.Direction.REVERSE);
        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.REVERSE);


        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Odo_raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Odo_raise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Top_Gripper.setPosition(0);
        Base_Gripper.setPosition(0.4);
        Base_Pivot.setPosition(0.72);
        Top_Pivot.setPosition(0.4);

        drive.init(hardwareMap, 1);
    }

    public void Drive_To_Pos_1() {
        drive.TurnDegreesLeft(14);

        Top_Pivot.setPosition(0.4);

        Base_Pivot.setPosition(1);

        drive.StrafeDistance_Left(18, .5);

        drive.DriveDistanceLongReverse(22, .5);

        drive.StrafeDistance_Left(60, .5);

        drive.TurnToHeading(90,0.45);

        drive.StrafeDistance_Left(60, .5);
    }

    public void Drive_To_Pos_2() {
        drive.TurnDegreesLeft(14);

        Top_Pivot.setPosition(0.4);

        Base_Pivot.setPosition(1);

        drive.StrafeDistance_Left(18, .5);

        drive.DriveDistanceLongReverse(22, .5);

        drive.StrafeDistance_Left(60, .5);

        drive.TurnToHeading(90,0.45);
    }

    public void Drive_To_Pos_3() {
        drive.TurnDegreesLeft(14);

        Top_Pivot.setPosition(0.4);

        Base_Pivot.setPosition(1);

        drive.StrafeDistance_Left(18, .5);

        drive.DriveDistanceLongReverse(22, .5);

        drive.StrafeDistance_Left(60, .5);

        drive.TurnToHeading(90,0.45);

        drive.StrafeDistance(50, .5);
    }

    public void Drive_To_Destack() {
//        drive.DriveDistanceLong(140, 0.6);
//        Base_Pivot.setPosition(1);
//        drive.TurnDegrees(90);
//
//        drive.StrafeDistance_Left(25, 0.6);
//
//        drive.DriveDistance(24, 0.6);
//
//        drive.TurnDegrees(16);
//
//        drive.StrafeDistance(15, 0.6);
//
//        drive.DriveDistance(4, 0.6);

        drive.DriveDistanceLong(140, 0.6);

        Base_Pivot.setPosition(1);

        drive.TurnDegreesLeft(90);

        drive.DriveDistance(25, 0.6);

        drive.ResetEncoders();

        drive.StrafeDistance(15, 0.6);

        drive.TurnDegreesLeft(18);
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

    public void StrafeOdometry(double Distance, double power) {

        double CurrentPos = 0;

        double error = 1;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double StartingX = 0;

        double Distance_to_travel = 0;

        CurrentPos = getYpos();

        CurrentPosStarting = getYpos();

        StartingHeading = Math.toDegrees(getheading());

        StartingX = getXpos();

        Distance_to_travel = Distance - error;


        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (CurrentPos < Distance_to_travel - 1 + CurrentPosStarting || CurrentPos > Distance_to_travel + 1 + CurrentPosStarting) {

            telemetry.addData("Robot Position", odometry.getPose());
            telemetry.update();
            odometry.updatePose();
            CurrentPos = getYpos();
            if (CurrentPos < Distance_to_travel) {

                drive.RF.setPower(1.3*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-1.3*power);
                drive.LB.setPower(power);

            } else if (CurrentPos > Distance_to_travel) {

                drive.RF.setPower(-1.3*power);
                drive.RB.setPower(power);
                drive.LF.setPower(1.3*power);
                drive.LB.setPower(-power);
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        try {
            Thread.sleep(50);
        } catch (
                Exception e) {
            System.out.println(e.getMessage());
        }
        power = 0.2;
        odometry.updatePose();
        while (CurrentPos < Distance_to_travel - 0.1 + CurrentPosStarting || CurrentPos > Distance_to_travel + 0.1 + CurrentPosStarting) {

            telemetry.addData("Robot Position", odometry.getPose());
            telemetry.update();
            odometry.updatePose();
            CurrentPos = getYpos();
            if (CurrentPos < Distance_to_travel) {

                drive.RF.setPower(1.3*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-1.3*power);
                drive.LB.setPower(power);

            } else if (CurrentPos > Distance_to_travel) {

                drive.RF.setPower(-1.3*power);
                drive.RB.setPower(power);
                drive.LF.setPower(1.3*power);
                drive.LB.setPower(-power);
            }
        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        try {
            Thread.sleep(100);
        } catch (
                Exception e) {
            System.out.println(e.getMessage());
        }
        power = 0.2;
        odometry.updatePose();

        //turn
        while(StartingHeading-0.1 > Math.toDegrees(getheading())||StartingHeading+0.1 < Math.toDegrees(getheading())) {

            odometry.updatePose();

            if (StartingHeading > Math.toDegrees(getheading())) {
                drive.RF.setPower(-power);
                drive.RB.setPower(-power);
                drive.LF.setPower(power);
                drive.LB.setPower(power);
            } else if (StartingHeading < Math.toDegrees(getheading())) {
                drive.RF.setPower(power);
                drive.RB.setPower(power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-power);
            } else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        power = 0.12;
        odometry.updatePose();

        //Drive
        while(StartingX - 0.1 > getXpos() ||StartingX + 0.1 < getXpos()){

            odometry.updatePose();

            if (StartingX > getXpos()) {
                drive.RF.setPower(power);
                drive.RB.setPower(power);
                drive.LF.setPower(power);
                drive.LB.setPower(power);

            } else if (StartingX < getXpos()) {

                drive.RF.setPower(-power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        power = 0.2;
        odometry.updatePose();

        //Turn
        while(StartingHeading-0.1>Math.toDegrees(getheading())||StartingHeading+0.1 <Math.toDegrees(getheading())) {

            odometry.updatePose();

            if (StartingHeading > Math.toDegrees(getheading())) {
                drive.RF.setPower(-power);
                drive.RB.setPower(-power);
                drive.LF.setPower(power);
                drive.LB.setPower(power);
            } else if (StartingHeading < Math.toDegrees(getheading())) {
                drive.RF.setPower(power);
                drive.RB.setPower(power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-power);
            } else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void StrafeOdometryNoCorrrection(double Distance, double power) {

        double CurrentPos = 0;

        double error = 1;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double StartingX = 0;

        double Distance_to_travel = 0;

        CurrentPos = getYpos();

        CurrentPosStarting = getYpos();

        StartingHeading = Math.toDegrees(getheading());

        StartingX = getXpos();

        Distance_to_travel = Distance - error;


        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (CurrentPos < Distance_to_travel - 1 + CurrentPosStarting || CurrentPos > Distance_to_travel + 1 + CurrentPosStarting) {

            telemetry.addData("Robot Position", odometry.getPose());
            telemetry.update();
            odometry.updatePose();
            CurrentPos = getYpos();
            if (CurrentPos < Distance_to_travel) {

                drive.RF.setPower(1.3*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-1.3*power);
                drive.LB.setPower(power);

            } else if (CurrentPos > Distance_to_travel) {

                drive.RF.setPower(-1.3*power);
                drive.RB.setPower(power);
                drive.LF.setPower(1.3*power);
                drive.LB.setPower(-power);
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
//        try {
//            Thread.sleep(50);
//        } catch (
//                Exception e) {
//            System.out.println(e.getMessage());
//        }
//        power = 0.2;
//        odometry.updatePose();
//        while (CurrentPos < Distance_to_travel - 0.1 + CurrentPosStarting || CurrentPos > Distance_to_travel + 0.1 + CurrentPosStarting) {
//
//            telemetry.addData("Robot Position", odometry.getPose());
//            telemetry.update();
//            odometry.updatePose();
//            CurrentPos = getYpos();
//            if (CurrentPos < Distance_to_travel) {
//
//                drive.RF.setPower(1.3*power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(-1.3*power);
//                drive.LB.setPower(power);
//
//            } else if (CurrentPos > Distance_to_travel) {
//
//                drive.RF.setPower(-1.3*power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(1.3*power);
//                drive.LB.setPower(-power);
//            }
//        }
//
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);
//        try {
//            Thread.sleep(100);
//        } catch (
//                Exception e) {
//            System.out.println(e.getMessage());
//        }
//        power = 0.2;
//        odometry.updatePose();
//
//        //turn
//        while(StartingHeading-0.1 > Math.toDegrees(getheading())||StartingHeading+0.1 < Math.toDegrees(getheading())) {
//
//            odometry.updatePose();
//
//            if (StartingHeading > Math.toDegrees(getheading())) {
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(power);
//            } else if (StartingHeading < Math.toDegrees(getheading())) {
//                drive.RF.setPower(power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-power);
//            } else {
//                drive.RF.setPower(0);
//                drive.RB.setPower(0);
//                drive.LF.setPower(0);
//                drive.LB.setPower(0);
//            }
//
//        }
//
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);
//
//        power = 0.12;
//        odometry.updatePose();
//
//        //Drive
//        while(StartingX - 0.1 > getXpos() ||StartingX + 0.1 < getXpos()){
//
//            odometry.updatePose();
//
//            if (StartingX > getXpos()) {
//                drive.RF.setPower(power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(power);
//
//            } else if (StartingX < getXpos()) {
//
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-power);
//            }else {
//                drive.RF.setPower(0);
//                drive.RB.setPower(0);
//                drive.LF.setPower(0);
//                drive.LB.setPower(0);
//            }
//
//        }
//
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);
//
//        power = 0.2;
//        odometry.updatePose();
//
//        //Turn
//        while(StartingHeading-0.1>Math.toDegrees(getheading())||StartingHeading+0.1 <Math.toDegrees(getheading())) {
//
//            odometry.updatePose();
//
//            if (StartingHeading > Math.toDegrees(getheading())) {
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(power);
//            } else if (StartingHeading < Math.toDegrees(getheading())) {
//                drive.RF.setPower(power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-power);
//            } else {
//                drive.RF.setPower(0);
//                drive.RB.setPower(0);
//                drive.LF.setPower(0);
//                drive.LB.setPower(0);
//            }
//
//        }
//
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);

    }

    public void DriveOdometry(double Distance, double power){
        double CurrentPos = 0;

        double error = 0;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double StartingY = 0;

        double Distance_to_travel = 0;

        CurrentPos = getXpos();

        CurrentPosStarting = getYpos();

        StartingHeading = Math.toDegrees(getheading());

        StartingY = getYpos();

        Distance_to_travel = Distance - error + 0.48;


        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(CurrentPos < Distance_to_travel - 1 + CurrentPosStarting|| CurrentPos > Distance_to_travel + 1 + CurrentPosStarting){

            telemetry.addData("Robot Position", odometry.getPose());
            telemetry.update();
            odometry.updatePose();
            CurrentPos = getXpos();
            if(CurrentPos > (Distance_to_travel + CurrentPosStarting)*0.8 || CurrentPos > (Distance_to_travel + CurrentPosStarting)*0.8){
                power = 0.12;
            }
            if (CurrentPos < Distance_to_travel){

                drive.RF.setPower(power);
                drive.RB.setPower(0.2*power);
                drive.LF.setPower(power);
                drive.LB.setPower(0.2*power);

            }else if(CurrentPos > Distance_to_travel){

                drive.RF.setPower(-power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-power);
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        power = 0.09;

        while(CurrentPos < Distance_to_travel - 0.05 + CurrentPosStarting|| CurrentPos > Distance_to_travel + 0.05 + CurrentPosStarting){

            telemetry.addData("Robot Position", odometry.getPose());
            telemetry.update();
            odometry.updatePose();
            CurrentPos = getXpos();
            if (CurrentPos < Distance_to_travel){

                drive.RF.setPower(power);
                drive.RB.setPower(0.2*power);
                drive.LF.setPower(power);
                drive.LB.setPower(0.2*power);

            }else if(CurrentPos > Distance_to_travel){

                drive.RF.setPower(-power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-power);
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        power = 0.25;
        odometry.updatePose();

        //turn
        while (StartingHeading-0.1 > Math.toDegrees(getheading()) || StartingHeading+0.1 < Math.toDegrees(getheading())){

            odometry.updatePose();

            if (StartingHeading > Math.toDegrees(getheading())){
                drive.RF.setPower(-power);
                drive.RB.setPower(-0.3*power);
                drive.LF.setPower(power);
                drive.LB.setPower(0.3*power);
            }else if (StartingHeading < Math.toDegrees(getheading())){
                drive.RF.setPower(power);
                drive.RB.setPower(0.3*power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-0.3*power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        power = 0.29;

        while (StartingY - 0.1 > getYpos() || StartingY + 0.1 < getYpos()){

            odometry.updatePose();

            if (StartingY > getYpos()){
                drive.RF.setPower(power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-power);
                drive.LB.setPower(power);

            }else if (StartingY < getYpos()){

                drive.RF.setPower(-power);
                drive.RB.setPower(power);
                drive.LF.setPower(power);
                drive.LB.setPower(-power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        power = 0.25;
        odometry.updatePose();

        while (StartingHeading-0.1 > Math.toDegrees(getheading()) || StartingHeading+0.1 < Math.toDegrees(getheading())){

            odometry.updatePose();

            if (StartingHeading > Math.toDegrees(getheading())){
                drive.RF.setPower(-power);
                drive.RB.setPower(-0.3*power);
                drive.LF.setPower(power);
                drive.LB.setPower(0.3*power);
            }else if (StartingHeading < Math.toDegrees(getheading())){
                drive.RF.setPower(power);
                drive.RB.setPower(0.3*power);
                drive.LF.setPower(-power);
                drive.LB.setPower(-0.3*power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void TurnOdometry(double Degrees, double power){
        double CurrentPos = 0;

        double error = 0;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double StartingY = 0;

        double Degrees_to_turn = 0;

        CurrentPos = getXpos();

        CurrentPosStarting = getYpos();

        StartingHeading = Math.toDegrees(getheading());

        StartingY = getYpos();

        double StartingX = getXpos();

        Degrees_to_turn = Degrees - error;


        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometry.updatePose();

        while (Degrees_to_turn - 1 > Math.toDegrees(getheading()) +  StartingHeading || Degrees_to_turn + 1 < Math.toDegrees(getheading() + StartingHeading)){

            odometry.updatePose();

            if (Degrees_to_turn > Math.toDegrees(getheading())){
                drive.RF.setPower(-1.2*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(1.2*power);
                drive.LB.setPower(power);
            }else if (Degrees_to_turn < Math.toDegrees(getheading())){
                drive.RF.setPower(1.2*power);
                drive.RB.setPower(power);
                drive.LF.setPower(-1.2*power);
                drive.LB.setPower(-power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }


        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        power = 0.15;

        while (Degrees_to_turn - 0.1 > Math.toDegrees(getheading()) + StartingHeading|| Degrees_to_turn + 0.1 < Math.toDegrees(getheading() + StartingHeading)){

            odometry.updatePose();

            if (Degrees_to_turn > Math.toDegrees(getheading())){
                drive.RF.setPower(-1.2*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(1.2*power);
                drive.LB.setPower(power);
            }else if (Degrees_to_turn < Math.toDegrees(getheading())){
                drive.RF.setPower(1.2*power);
                drive.RB.setPower(power);
                drive.LF.setPower(-1.2*power);
                drive.LB.setPower(-power);
            }else {
                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
    }


    public void DropPreLoad() {
        Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        Right_Slide.setTargetPosition(2000);
        Left_Slide.setTargetPosition(2000);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Right_Slide.isBusy() && Left_Slide.isBusy()) {
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
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        Top_Gripper.setPosition(0.3);
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        //TO DO: Insert WHILE loop
        if (Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(100);
            } catch (Exception e) {
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
    }

    public void CheckVSlidePos() {
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
    }

    public void Destack(double De_pos) {

        Base_Gripper.setPosition(0.4);

        Destacker_Left.setPosition(De_pos);
        Destacker_Right.setPosition(De_pos);

        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0.1);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Extend.setPower(-1);

        conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

        //extend till we find a cone or get to the slides limit
        while (!conefound && Extend.getCurrentPosition() > -1930) {

            CheckVSlidePos();

            conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

            Extend.setPower(-1);

        }

        Extend.setPower(0);

        if (conefound) {

            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //make sure gripper is closed
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if (Base_Gripper.getPosition() == 0) {

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(0.9);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(0.8);
                Destacker_Right.setPosition(0.8);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);

                try {
                    Thread.sleep(600);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Nest_Occupied = colour.blue() > 3000;

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }


                if (Nest_Occupied) {

                    //open top gripper
                    Top_Gripper.setPosition(0.35);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot to pick up the cone
                    Top_Pivot.setPosition(1);

                    if (Base_Pivot.getPosition() > 0.9) {

                        try {
                            Thread.sleep(500);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }

                        //close top gripper
                        Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(120);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }

                        //take top pivot over
                        Top_Pivot.setPosition(0.5);

                        try {
                            Thread.sleep(200);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }

                        //put base pivot back to zero
                        Base_Pivot.setPosition(0.1);

                    }

                }else {
                    //Abort
                    abort = true;

                }

                try {
                    Thread.sleep(400);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Nest_Occupied = colour.blue() > 3000;

                if(!Nest_Occupied){
                    Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    Base_Pivot.setPosition(0.1);

                    //Extend vertical slides and drop cone
                    Right_Slide.setTargetPosition(2000);
                    Left_Slide.setTargetPosition(2000);
                    Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Right_Slide.isBusy() && Left_Slide.isBusy()) {
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
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Gripper.setPosition(0.3);

                    try {
                        Thread.sleep(400);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //TO DO: Insert WHILE loop
                        if (Top_Gripper.getPosition() == 0.3) {
                            try {
                                Thread.sleep(250);
                            } catch (Exception e) {
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

                    }else {
                        //Abort
                        abort = true;
                    }

                }

            }else {
                abort = true;
                Top_Pivot.setPosition(0.5);
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Extend.setPower(0.8);
                }
                Extend.setPower(0);
                Base_Pivot.setPosition(0.1);
                Destacker_Left.setPosition(0.8);
                Destacker_Right.setPosition(0.8);
            }
            try {
                Thread.sleep(20);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

        }

    public void Destack_5 () {
        Base_Gripper.setPosition(0.4);

        //cone 1
        Destack(De_Pos_1);

        if (abort){
            //Drive to position
            Top_Pivot.setPosition(Top_Pivot_Waiting);
            Base_Pivot.setPosition(1);
        }else {

            //cone 2
            Destack(De_Pos_2);

            if (abort){
                //Drive to position
                Top_Pivot.setPosition(Top_Pivot_Waiting);
                Base_Pivot.setPosition(1);
            }else {

                //cone 3
                Destack(De_Pos_3);

                if (abort){
                    //Drive to position
                    Top_Pivot.setPosition(Top_Pivot_Waiting);
                    Base_Pivot.setPosition(1);
                }else {
                    //cone 4
                    Destack(De_Pos_4);

                    if (abort){
                        //Drive to position
                        Top_Pivot.setPosition(Top_Pivot_Waiting);
                        Base_Pivot.setPosition(1);
                    }else {

                        //cone 5
                        Destack(De_Pos_5);
                    }
                }

            }

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
}
