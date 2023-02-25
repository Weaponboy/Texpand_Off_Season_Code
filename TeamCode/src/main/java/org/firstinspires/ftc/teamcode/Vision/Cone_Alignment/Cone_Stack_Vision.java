package org.firstinspires.ftc.teamcode.Vision.Cone_Alignment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;


@Autonomous
@Config
public class Cone_Stack_Vision extends OpMode {

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

    Drivetrain drive = new Drivetrain();
    private double power;
    private MotorEx LF, RF, LB, RB;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    private ElapsedTime runtime = new ElapsedTime();

    Red_Cone_Pipe Cone_Pipeline;
    private int loop1 = 0;
    private OpenCvCamera webcam;

    OpenCvWebcam Texpandcamera;

    public double Distance_To_Travel;

    public double CenterOfScreen = 330;

    public double rectPositionFromLeft = 0;

    public static int Gain = 1;
    public static int Exposure = 8;
    public void init() {

        drive.init(hardwareMap);

        Cone_Pipeline = new Red_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        Texpandcamera.setPipeline(Cone_Pipeline);

        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(Exposure, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(Gain);

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

        // Set the pipeline depending on id
        Texpandcamera.setPipeline(Cone_Pipeline);

        FtcDashboard.getInstance().startCameraStream(Texpandcamera,30);
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

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.update(0, 0, 0);

        drive.init(hardwareMap);


    }


    @Override
    public void init_loop() {

        drive.init(hardwareMap);

//        //Calculate distance to drive to aline
        rectPositionFromLeft = Cone_Pipeline.getRectX();
//
        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen;

        Distance_To_Travel = Distance_To_Travel / 20;

        //Telemetry to be displayed during init_loop()
        telemetry.addData("FPS", Texpandcamera.getFps());
        telemetry.addData("S", Cone_Pipeline.getS());
        telemetry.addData("V", Cone_Pipeline.getV());
        telemetry.addData("H", Cone_Pipeline.getH());
        telemetry.addData("Min H Value", Blue_Cone_Pipe.Min_H);
        telemetry.addData("con", Cone_Pipeline.numcontours());
        telemetry.addData("rects", Cone_Pipeline.getRects());
        telemetry.addData("Target cm", Distance_To_Travel);
        telemetry.addData("Cone Position", Cone_Pipeline.getRectX());
        telemetry.addData("rect X", Cone_Pipeline.getRectX());
        telemetry.addData("rect Y", Cone_Pipeline.getRectY());
        telemetry.addData("Target CM", Distance_To_Travel);
        telemetry.update();
    }

    @Override
    public void loop() {
        double power;

        rectPositionFromLeft = 0;

        Distance_To_Travel = 0;

        CenterOfScreen = 309;


        if(loop1 == 0){
            power = 0.15;
            drive.DriveEncoders();

            rectPositionFromLeft = Cone_Pipeline.getRectX();
            while (Math.abs(rectPositionFromLeft - CenterOfScreen) > 80){

                telemetry.addData("rect X", Cone_Pipeline.getRectX());
                telemetry.addData("rect Y", Cone_Pipeline.getRectY());
                telemetry.addData("rect1", "1st loop");
                telemetry.update();

                if (rectPositionFromLeft < CenterOfScreen) {
                    CenterOfScreen = 329;
                    drive.RF.setPower(-power);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power);
                    drive.LB.setPower(-power);
                    rectPositionFromLeft = Cone_Pipeline.getRectX();

                } else if (rectPositionFromLeft > CenterOfScreen) {
                    CenterOfScreen = 309;
                    drive.RF.setPower(power);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power);
                    drive.LB.setPower(power);
                    rectPositionFromLeft = Cone_Pipeline.getRectX();
                }


            }

            power = 0.11;
            drive.DriveEncoders();

            drive.RF.setPower(-0.05);
            drive.RB.setPower(-0.05);
            drive.LF.setPower(-0.05);
            drive.LB.setPower(-0.05);
            try {
                Thread.sleep(200);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            rectPositionFromLeft = Cone_Pipeline.getRectX();

            while (Math.abs(rectPositionFromLeft - CenterOfScreen) > 10){

                telemetry.addData("rect X", Cone_Pipeline.getRectX());
                telemetry.addData("rect Y", Cone_Pipeline.getRectY());
                telemetry.addData("rect", "2nd loop");
                telemetry.update();


                if (rectPositionFromLeft < CenterOfScreen - 3*0) {
                    CenterOfScreen = 329;
                    //left
                    drive.RF.setPower(-power);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power);
                    drive.LB.setPower(-power);
                    rectPositionFromLeft = Cone_Pipeline.getRectX();

                } else if (rectPositionFromLeft >  CenterOfScreen + 3*0) {
                    CenterOfScreen = 309;
                    //right
                    drive.RF.setPower(power);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power);
                    drive.LB.setPower(power);
                    rectPositionFromLeft = Cone_Pipeline.getRectX();

                }else{
                    drive.RF.setPower(-0.05);
                    drive.RB.setPower(-0.05);
                    drive.LF.setPower(-0.05);
                    drive.LB.setPower(-0.05);
                }


            }
            drive.RF.setPower(-0.05);
            drive.RB.setPower(-0.05);
            drive.LF.setPower(-0.05);
            drive.LB.setPower(-0.05);
            try {
                Thread.sleep(70);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);
        }
        loop1 = 1;


        rectPositionFromLeft = Cone_Pipeline.getRectX();

        telemetry.addData("rect X", Cone_Pipeline.getRectX());
        telemetry.addData("rect Y", Cone_Pipeline.getRectY());
        telemetry.update();


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

}
