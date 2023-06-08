package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeP;

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
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Base_Gripper;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Top_Gripper;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


@Autonomous
public class Blue_Left_High_Cycle extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

    public static final double CENTER_WHEEL_OFFSET = 0;

    public static final double WHEEL_DIAMETER = 3.5;

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean lowering = false;

    private boolean Nest_Occupied = false;

    private double counterfornest = 0;

    private boolean Extending_High = false;

    private MecanumDrive driveTrain;

    private boolean conefound = false;

    private boolean abort = false;

    private MotorEx LF, RF, LB, RB;

    Setpoints setpoints = new Setpoints();

    Gyro gyro;

    PIDFController drivePID;
    PIDFController strafePID;
    PIDFController PivotPID;

    private boolean time = false;

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

    Top_Gripper top = new Top_Gripper();

    Base_Gripper bottom = new Base_Gripper();

    double CurrentXPos = 0;
    double CurrentYPos = 0;

    double StartingHeading = 0;

    double StartingHeadinggyro = 0;

    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;

    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.32;


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

        OdometryInit();

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();

        waitForStart();

        while (opModeIsActive()){

//            double leftpodread = leftOdometer.getPosition();
//            double rightpodread = rightOdometer.getPosition();
//            double centerpodread = centerOdometer.getPosition();
//
//            odometry.update(leftpodread, rightpodread, centerpodread);

            odometry.updatePose();

            telemetry.addData("LF", LF.encoder.getPosition());
            telemetry.addData("left odo", leftOdometer.getPosition());
            telemetry.addData("LB", LB.encoder.getPosition());
            telemetry.addData("center odo", centerOdometer.getPosition());
            telemetry.addData("RF", RF.encoder.getPosition());
            telemetry.addData("right odo", rightOdometer.getPosition());
            telemetry.addData("heading", Math.toDegrees(getheading()));
            telemetry.addData("X", getXpos());
            telemetry.addData("Y", getYpos());
            telemetry.update();

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

        driveTrain = new MecanumDrive(RB, RF, LB, LF);

        leftOdometer = LB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = RF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = LF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                centerOdometer::getDistance,
                rightOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0)));

//        odometry.updatePose();

    }


}

