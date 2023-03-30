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

package org.firstinspires.ftc.teamcode.Auto.Test_Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
@Disabled
public class Blue_A2_Start_High extends LinearOpMode {

    public double Distance_To_Travel;

    public BNO055IMU imu         = null;
    public double Degrees_To_Turn;

    public Orientation yawAngle;

    public double ConversionPixelstoCm = 22;//need to tune this

    public double CenterOfScreen = 300;

    private  double poleDistance = 0;

    private  double TargetPoleDistance = 42;

    private  double TravelDistance = 0;

    private DistanceSensor Back_Distance;

    Pole_Pipe Pole;

    private DistanceSensor sensorRange;
    Blue_Cone_Pipe Cone_Pipeline;
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    double Top_Open_Wide = 0.36;
    double Top_Open = 0.33;
    double Top_Closed = 0;

    double Top_Pivot_Collect = 1;
    double Top_Pivot_Waiting = 0.6;
    double Top_Pivot_Ready_Drop = 0.3;
    double Top_Pivot_Drop_Off = 0;
    public double poweradj = 0.085;


    public double rectPositionFromLeft = 0;

    private double Cone_power;

    public ColorSensor colour = null;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

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


    // destacker positions
    double De_Pos_1 = 0;
    double De_Pos_2 = 0.36;
    double De_Pos_3 = 0.49;
    double De_Pos_4 = 0.68;
    double De_Pos_5 = 0.72;

    OpenCvCamera camera;
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

    private ExecutorService executor;
    private AtomicInteger counter;

    AprilTagDetection tagOfInterest = null;
    public OpenCvWebcam Texpandcamera;

    public OpenCvWebcam Frontcam;

    @Override
    public void runOpMode() throws InterruptedException {

        executor = Executors.newFixedThreadPool(2);

        // Create a shared counter that is incremented by the threads
        counter = new AtomicInteger();

        initialize();

        Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.init(hardwareMap, 1);

        Cone_Pipeline = new Blue_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        Frontcam  = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"), viewportContainerIds[1]);


        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        drive.init(hardwareMap, 1);

        Texpandcamera.setPipeline(aprilTagDetectionPipeline);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Frontcam.setPipeline(Cone_Pipeline);

        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(25, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(50);

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
        Frontcam.setPipeline(Cone_Pipeline);

        Texpandcamera.setPipeline(aprilTagDetectionPipeline);

        Frontcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Frontcam.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Frontcam.getExposureControl().setExposure(25, TimeUnit.MILLISECONDS);

                Frontcam.getGainControl().setGain(50);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                Frontcam.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    Frontcam.getFocusControl().setFocusLength(400);
                }

                Frontcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) { }
        });

        Frontcam.setPipeline(Cone_Pipeline);

        // Set the pipeline depending on id
        Texpandcamera.setPipeline(aprilTagDetectionPipeline);
        FtcDashboard.getInstance().startCameraStream(Texpandcamera,30);

        telemetry.setMsTransmissionInterval(50);

        drive.init(hardwareMap, 1);


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

        Pole = new Pole_Pipe();

        Texpandcamera.setPipeline(Pole);

        if (tagOfInterest.id == RIGHT) {
            //Position 3
            telemetry.addData("Stop Position", "3");
            telemetry.update();

            Drive_To_Destack();

            Base_Pivot.setPosition(0.05);

            Base_Gripper.setPosition(0.4);

            DropPreLoad();

            Destack_3();

//            Drive_To_Pos_3();

        } else if (tagOfInterest.id == LEFT) {
            //Position 1
            telemetry.addData("Stop Position", "1");
            telemetry.update();

//            Drive_To_Destack();
//
//            DropPreLoad();
//
            Destack_3();
//
//            Drive_To_Pos_1();

        } else if (tagOfInterest.id == MIDDLE) {
            //Position 2
            Texpandcamera.setPipeline(Pole);

            Reverse_To_Destack();

            Base_Gripper.setPosition(0.4);

            Destack_3();

            Drive_To_Pos_2();

        }

        executor.shutdown();
        executor.awaitTermination(1, TimeUnit.MINUTES);

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
        colour = hardwareMap.get(ColorSensor.class, "colour");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

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

        Top_Gripper.setPosition(0);
        Base_Gripper.setPosition(0.4);
        Base_Pivot.setPosition(0.72);

    }

    public void Drive_To_Pos_1() {

//        drive.TurnToHeadingFast(90,0.8);

        drive.DriveDistanceRamp(-65, 0.6);

    }

    public void Drive_To_Pos_2() {

//        drive.TurnToHeadingFast(90,0.8);

        drive.DriveDistanceRamp(-5, 0.6);

    }

    public void Drive_To_Pos_3() {

//        drive.TurnToHeadingFast(90,0.8);

        drive.DriveDistanceRamp(60, 0.6);

    }

    public void Drive_To_Destack() {

        drive.DriveDistanceRamp(131,0.7);
        Base_Pivot.setPosition(0.85);
        Top_Pivot.setPosition(0.4);

        drive.TurnToHeading(-90,0.45);

        drive.DriveDistanceRamp(4,0.2);

        drive.ResetEncoders();

        drive.TurnToHeading(-130,0.45);

    }

    public void Reverse_To_Destack() {

        drive.DriveDistanceRamp(-128, 0.7);

        Base_Pivot.setPosition(0.85);

        Top_Pivot.setPosition(0.4);

        drive.TurnToHeadingFast(50, 1);

        PoleAlignment();

        DropPreLoad();

        drive.TurnToHeadingFast(90, 0.8);

        drive.DriveDistanceRamp(10, 0.2);

        drive.ResetEncoders();
    }

    public void DropPreLoad() {
        Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        Right_Slide.setTargetPosition(1800);
        Left_Slide.setTargetPosition(1800);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Right_Slide.getCurrentPosition() < 1750 && Left_Slide.getCurrentPosition() < 1750) {
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
            Top_Pivot.setPosition(0.42);
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

        //TO DO: Insert WHILE loop
        if (Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(200);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Top_Pivot.setPosition(0.4);
            try {
                Thread.sleep(300);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
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

    public void Destack(double De_pos, boolean reverse) {

        Base_Gripper.setPosition(0.4);

        Destacker_Left.setPosition(De_pos);
        Destacker_Right.setPosition(De_pos);

        if(Destacker_Left.getPosition() == De_Pos_1){
            Base_Pivot.setPosition(0.1);
        }else{
            Base_Pivot.setPosition(0.05);
        }


        Top_Pivot.setPosition(0.5);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Extend.setPower(-1);

        conefound = sensorRange.getDistance(DistanceUnit.MM) < 60;

        //extend till we find a cone or get to the slides limit
        while (!conefound && Extend.getCurrentPosition() > -900) {

            CheckVSlidePos();

            conefound = sensorRange.getDistance(DistanceUnit.MM) < 60;

            boolean SlowPoint = Extend.getCurrentPosition() < -530;

            if (SlowPoint){
                Extend.setPower(-0.35);
            }else {
                Extend.setPower(-1);
            }

        }
        Extend.setPower(0);

        if (!conefound){
            try {
                Thread.sleep(500);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        if (conefound){

            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //make sure gripper is closed
            try {
                Thread.sleep(150);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            Base_Pivot.setPosition(0.82);

            try {
                Thread.sleep(300);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            executor.submit(() -> {
                if(reverse){
                    drive.TurnToHeadingFast(55, 1);
                }
            });

            //if the base gripper is closed retract the horizontal slides
            if (Base_Gripper.getPosition() == 0) {

                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (Extend.isBusy()) {

                    CheckVSlidePos();

                    Base_Pivot.setPosition(0.82);

                    Extend.setPower(0.6);

                    if(Extend.getCurrentPosition() > -300){
                        Destacker_Left.setPosition(De_Pos_5);
                        Destacker_Right.setPosition(De_Pos_5);
                    }
                    if(Extend.getCurrentPosition() > -100){
                        Base_Gripper.setPosition(0.4);
                    }
                    if(Extend.getCurrentPosition() > -50){
                        //open top gripper
                        Top_Gripper.setPosition(0.35);

                        //take top pivot to pick up the cone
                        Top_Pivot.setPosition(1);
                    }
                }

                Extend.setPower(0);


                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);

                Nest_Occupied = colour.blue() > 2000;

                Base_Pivot.setPosition(1);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }


                Nest_Occupied = colour.blue() > 2000;

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

                Nest_Occupied = colour.blue() > 2000;

                if(!Nest_Occupied){
//                    Top_Pivot.setPosition(0.8);
                    try {
                        Thread.sleep(250);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }
//                    Top_Pivot.setPosition(1);
                }

                Nest_Occupied = colour.blue() > 2000;


                if (Nest_Occupied) {

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero

                    Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    //Extend vertical slides and drop cone
                    Right_Slide.setTargetPosition(1800);
                    Left_Slide.setTargetPosition(1800);
                    Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Right_Slide.getCurrentPosition() < 1750 && Left_Slide.getCurrentPosition() < 1750) {
                        Right_Slide.setPower(1);
                        Left_Slide.setPower(1);

                        for (int i = 1; i <= 50; i++) {
                            rectPositionFromLeft = Pole.TargetHighrectX;
                        }

                        if (rectPositionFromLeft > -1) {
                            Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

                            Degrees_To_Turn = Degrees_To_Turn / 20;

                            drive.yawAngle = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }

//                        TravelDistance = Back_Distance.getDistance(DistanceUnit.CM) - TargetPoleDistance;
//
//                        drive.DriveDistanceRamp(-TravelDistance, 0.2);

                        Top_Pivot.setPosition(0.42);


                    }
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);


                    if(Degrees_To_Turn != 0) {
                        drive.TurnToHeading(drive.yawAngle.firstAngle + Degrees_To_Turn , 0.3);
                        Degrees_To_Turn = 0;
                    }

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    if(Right_Slide.getCurrentPosition() > 1650){
                        Top_Pivot.setPosition(0);
                    }

                    Top_Pivot.setPosition(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    try {
                        Thread.sleep(275);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    executor.submit(() -> {
                        if(reverse){
                            drive.TurnToHeadingFast(90, 0.8);
                        }
                    });

                    Top_Gripper.setPosition(0.3);

                    //TO DO: Insert WHILE loop
                    if (Top_Gripper.getPosition() == 0.3) {
                        try {
                            Thread.sleep(200);
                        } catch (Exception e) {
                            System.out.println(e.getMessage());
                        }

                        Right_Slide.setTargetPosition(0);
                        Left_Slide.setTargetPosition(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);

                        Top_Pivot.setPosition(0.4);

                        lowering = true;
                    }


                }else {
                    //Abort
                    abort = true;
                }


            } else {
                //Abort
                abort = true;
            }

        }else{
            Extend.setTargetPosition(0);

            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (Extend.isBusy()) {

                CheckVSlidePos();
                Extend.setPower(0.8);

            }
            Extend.setPower(0);

            Destacker_Left.setPosition(De_Pos_5);
            Destacker_Right.setPosition(De_Pos_5);

            abort = true;

        }


    }

    public void PoleAlignment (){
        for (int i = 1; i <= 50; i++) {
            rectPositionFromLeft = Pole.TargetHighrectX;
        }

        if (rectPositionFromLeft > -1) {
            Degrees_To_Turn = rectPositionFromLeft - CenterOfScreen;

            Degrees_To_Turn = Degrees_To_Turn / 20;

            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

        try {
            Thread.sleep(200);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        if(Degrees_To_Turn != 0) {
            drive.TurnToHeading(yawAngle.firstAngle + Degrees_To_Turn, 0.3);
            Degrees_To_Turn = 0;
        }
    }

    public void coneAlignmentStrafe(){

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

    public void Destack_5 () {
        Base_Gripper.setPosition(0.4);

        //cone 1
        Base_Pivot.setPosition(0.12);
        Destack(De_Pos_1, false);
        Base_Pivot.setPosition(0.12);

        if (abort){
            //Drive to position
            Top_Pivot.setPosition(Top_Pivot_Waiting);
            Base_Pivot.setPosition(0.72);
        }else {

            //cone 2
            Base_Pivot.setPosition(0.12);
            Destack(De_Pos_2, false);
            Base_Pivot.setPosition(0.12);
            if (abort){
                //Drive to position
                Base_Pivot.setPosition(0.1);
                Top_Pivot.setPosition(Top_Pivot_Waiting);
                Base_Pivot.setPosition(0.72);
            }else {
                Base_Pivot.setPosition(0.12);
                //cone 3
                Destack(De_Pos_3, false);
                Base_Pivot.setPosition(0.12);
                if (abort){
                    //Drive to position
                    Top_Pivot.setPosition(Top_Pivot_Waiting);
                    Base_Pivot.setPosition(0.72);
                }else {
                    Base_Pivot.setPosition(0.12);
                    //cone 4
                    Destack(De_Pos_4, false);
                    Base_Pivot.setPosition(0.12);
                    if (abort){
                        //Drive to position
                        Top_Pivot.setPosition(Top_Pivot_Waiting);
                        Base_Pivot.setPosition(0.72);
                    }else {
                        Base_Pivot.setPosition(0.12);
                        //cone 5
                        Destack(De_Pos_5, false);
                        Base_Pivot.setPosition(0.12);
                    }
                }

            }

        }

    }

    public void Destack_4 () {
        Base_Gripper.setPosition(0.4);
        Base_Pivot.setPosition(0.12);
        //cone 1
        Destack(De_Pos_1, false);
        Base_Pivot.setPosition(0.2);

        if (abort){
            //Drive to position
            Top_Pivot.setPosition(Top_Pivot_Waiting);
            Base_Pivot.setPosition(0.72);
        }else {
            Base_Pivot.setPosition(0.12);
            //cone 2
            Destack(De_Pos_2, false);
            Base_Pivot.setPosition(0.12);
            if (abort){
                //Drive to position
                Base_Pivot.setPosition(0.1);
                Top_Pivot.setPosition(Top_Pivot_Waiting);
                Base_Pivot.setPosition(0.72);
            }else {
                Base_Pivot.setPosition(0.12);
                //cone 3
                Destack(De_Pos_3, false);
                Base_Pivot.setPosition(0.12);
                if (abort){
                    //Drive to position
                    Top_Pivot.setPosition(Top_Pivot_Waiting);
                    Base_Pivot.setPosition(0.72);
                }else {
                    Base_Pivot.setPosition(0.12);
                    //cone 4
                    Destack(De_Pos_4, false);
                    Base_Pivot.setPosition(0.12);
                    if (abort){
                        //Drive to position
                        Top_Pivot.setPosition(Top_Pivot_Waiting);
                        Base_Pivot.setPosition(0.72);
                    }
                }

            }

        }

    }

    public void Destack_3 () {

        Base_Gripper.setPosition(0.4);

        drive.TurnToHeadingFast(90, 0.8);

        //cone 1
        Base_Pivot.setPosition(0.05);

        coneAlignmentStrafe();

        Destack(De_Pos_1, true);

        if (abort){
            //Drive to position
            Top_Pivot.setPosition(Top_Pivot_Waiting);
            Base_Pivot.setPosition(0.85);
            drive.DriveDistanceRamp(-15,0.6);

        }else {
            Base_Pivot.setPosition(0.05);

            drive.TurnToHeadingFast(90, 0.8);

            coneAlignmentStrafe();

            //cone 2
            Destack(De_Pos_2, true);
            Base_Pivot.setPosition(0.05);

            if (abort){

                //Drive to position
                Top_Pivot.setPosition(Top_Pivot_Waiting);
                Base_Pivot.setPosition(0.85);

                drive.DriveDistanceRamp(-15,0.6);

            }else {
                Base_Pivot.setPosition(0.05);
                drive.TurnToHeadingFast(90, 0.8);

                coneAlignmentStrafe();

                //cone 3
                Destack(De_Pos_3, true);

                Base_Pivot.setPosition(1);

                Top_Pivot.setPosition(0.8);

                drive.TurnToHeadingFast(90, 1);

                if (abort){
                    drive.DriveDistanceRamp(-15,0.6);
                }

            }

        }

    }


}
