package org.firstinspires.ftc.teamcode.Auto.Blue_Auto.A2_Start;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.targetRot;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.targetX;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.targetY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.wolfpackmachina.bettersensors.HardwareMapProvider;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Red_Cone_Pipe;
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
public class Odometry_A2 extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

    Top_gripper top = new Top_gripper();

    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();

    Slides slide = new Slides();

    Pole_Pipe Pole;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

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

    private ExecutorService executor;
    private AtomicInteger counter;

    AprilTagDetection tagOfInterest = null;

    public OpenCvWebcam Texpandcamera;

    public static final double CENTER_WHEEL_OFFSET = -17;

    public static final double WHEEL_DIAMETER = 3.5;

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MecanumDrive driveTrain;

    private MotorEx LF, RF, LB, RB;

    Gyro gyro;

    PIDFController drivePID;
    PIDFController strafePID;
    PIDFController PivotPID;

    double Xdist = 0;
    double Ydist = 0;

    double rotdist = 0;

    double RRXdist = 0;
    double RRYdist = 0;
    double Horizontal = 0;
    double Vertical = 0;

    double Horizontal2 = 0;
    double Vertical2 = 0;

    double ConvertedHeading = 0;
    double Pivot = 0;

    double CurrentXPos = 0;
    double CurrentYPos = 0;

    double StartingHeading = 0;

    double StartingHeadinggyro = 0;

    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.32;


    @Override
    public void runOpMode() throws InterruptedException {
        Init();

        OdometryInit();

        executor = Executors.newFixedThreadPool(2);

        // Create a shared counter that is incremented by the threads
        counter = new AtomicInteger();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Backcam");

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

        if(tagOfInterest.id == RIGHT){
            //Position 3
            telemetry.addData("Stop Position", "3");
            telemetry.update();

            Reverse_To_Destack();

        }else if (tagOfInterest.id == LEFT){
            //Position 1
            telemetry.addData("Stop Position", "1");
            telemetry.update();

            Reverse_To_Destack();

        }else if (tagOfInterest.id == MIDDLE){
            //Position 2
            telemetry.addData("Stop Position", "2");
            telemetry.update();

            Reverse_To_Destack();

            telemetry.addData("X", Math.abs(Xdist) + 100);
            telemetry.update();

            try {
                Thread.sleep(8000);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

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

    public void Init(){
        drive.init(hardwareMap, 0);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        //Vision 1 = Blue cone pipeline
        slide.init(hardwareMap, 1);
    }

    public void backCam(){

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
        bottom.Base_Pivot.setPosition(0.85);
        top.Top_Pivot.setPosition(0.4);

        drive.TurnToHeading(-90,0.45);

        drive.DriveDistanceRamp(4,0.2);

        drive.ResetEncoders();

        drive.TurnToHeading(-130,0.45);

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

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(3.141)));

//        odometry.updatePose();

    }

    public void Reverse_To_Destack() {

        Odo_Drive(128, 0, 140);

//        drive.DriveDistanceRamp(-128, 0.7);
//
//        bottom.Base_Pivot.setPosition(0.85);
//
//        top.Top_Pivot.setPosition(0.4);
//
//        drive.TurnToHeadingFast(50, 1);
//
//        drive.TurnToHeadingFast(90, 0.8);
//
//        drive.DriveDistanceRamp(10, 0.2);
//
//        drive.ResetEncoders();
    }

    public void bothCameras (){
//        Pole = new Pole_Pipe();
//
//        Cone_Pipeline_Blue = new Blue_Cone_Pipe();
//
//        Cone_Pipeline_Red = new Red_Cone_Pipe();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
//                .splitLayoutForMultipleViewports(
//                        cameraMonitorViewId, //The container we're splitting
//                        2, //The number of sub-containers to create
//                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
//
//        FrontWeb  = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
//
//        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"), viewportContainerIds[1]);
//
//        FrontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//
//                if (vision == 1){
//                    FrontWeb.setPipeline(Cone_Pipeline_Blue);
//                } else if (vision == 2) {
//                    FrontWeb.setPipeline(Cone_Pipeline_Red);
//                }
//
//                FrontWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);
//
//                FrontWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);
//
//                FrontWeb.getGainControl().setGain(1);
//
//                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;
//
//                FrontWeb.getFocusControl().setMode(focusmode);
//
//                if (focusmode == FocusControl.Mode.Fixed){
//                    FrontWeb.getFocusControl().setFocusLength(450);
//                }
//
//                FrontWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode) { }
//        });
//        if (vision == 1){
//            FrontWeb.setPipeline(Cone_Pipeline_Blue);
//        } else if (vision == 2) {
//            FrontWeb.setPipeline(Cone_Pipeline_Red);
//        }
//
//
//        BackWeb.setPipeline(Pole);
//        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//
//
//                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);
//
//                BackWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);
//
//                BackWeb.getGainControl().setGain(1);
//
//                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;
//
//                BackWeb.getFocusControl().setMode(focusmode);
//
//                if (focusmode == FocusControl.Mode.Fixed){
//                    BackWeb.getFocusControl().setFocusLength(450);
//                }
//
//                BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode) { }
//        });
//        BackWeb.setPipeline(Pole);
//        FtcDashboard.getInstance().startCameraStream(BackWeb,30);
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot){

        do {

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
            Xdist = (targetX - CurrentXPos)*1.1;
            Ydist = (targetY - CurrentYPos)*1.1;

            //CONVERT HEADING FOR TRIG CALCS
            if(StartingHeading <= 0) {
                ConvertedHeading = (360 + StartingHeading);
            }else{
                ConvertedHeading = (0 + StartingHeading);
            }

            rotdist = (targetRot - ConvertedHeading);

            if(rotdist < -180) {
                rotdist = (360 + rotdist);
            }else if (rotdist > 180){
                rotdist = (rotdist - 360);
            }

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            RRXdist = Xdist*Math.cos(Math.toRadians(360-ConvertedHeading)) - Ydist*Math.sin(Math.toRadians(360-ConvertedHeading));

            RRYdist = Xdist*Math.sin(Math.toRadians(360-ConvertedHeading)) + Ydist*Math.cos(Math.toRadians(360-ConvertedHeading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = drivePID.calculate(-RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            //SET MOTOR POWER USING THE PID OUTPUT
            drive.RF.setPower(-Pivot + (Vertical + Horizontal));
            drive.RB.setPower((-Pivot*1.4) + (Vertical - (Horizontal*1.3)));
            drive.LF.setPower(Pivot + (Vertical - Horizontal));
            drive.LB.setPower((Pivot*1.4) + (Vertical + (Horizontal*1.3)));

        }while ((Math.abs(Xdist) > 0.5) || (Math.abs(Ydist) > 0.5) || (Math.abs(rotdist) > 0.5));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }
}
