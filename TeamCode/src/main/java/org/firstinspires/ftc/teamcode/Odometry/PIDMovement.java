package org.firstinspires.ftc.teamcode.Odometry;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.*;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

@TeleOp
public class PIDMovement extends OpMode {

    Gyro gyro = new Gyro("imu", 0);


    PIDFController drivePID;
    PIDFController strafePID;

    PIDFController PivotPID;

    double Xdist = 0;
    double Ydist = 0;
    double RRXdist = 0;
    double RRYdist = 0;
    double Horizontal = 0;
    double Vertical = 0;

    double ConvertedHeading = 0;
    double Pivot = 0;

    double CurrentXPos = 0;
    double CurrentYPos = 0;

    double StartingHeading = 0;

    double StartingHeadinggyro = 0;

    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.2  ;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static final double CENTER_WHEEL_OFFSET = -13;

    public static final double WHEEL_DIAMETER = 3.5;

    Drivetrain drive = new Drivetrain();

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MecanumDrive driveTrain;

    private MotorEx LF, RF, LB, RB;

    @Config
    public static class MovePIDTuning{
        public static double driveP = 0;
        public static double driveD = 0;
        public static double driveF = 0;


        public static double strafeP = 0;
        public static double strafeD = 0;
        public static double strafeF = 0;

        public static double rotationP = 0;
        public static double rotationD = 0;

        public static double rotationF = 0;

        public static double targetX, targetY, targetRot = 0;
    }

    @Override
    public void init() {

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(driveP, 0, driveD, driveF);

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

        gyro.sensorInit("imu");

        drive.init(hardwareMap);

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


    }

    @Override
    public void loop() {

        odometry.updatePose();

        CurrentXPos = getYpos();
        CurrentYPos = getXpos();
        StartingHeadinggyro = gyro.yaw();

        StartingHeading = Math.toDegrees(getheading());

        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gyro.update();

        drivePID.setPIDF(driveP, 0, driveD, driveF);

        strafePID.setPIDF(strafeP, 0, strafeD, strafeF);

        PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

        //set distance to travel error
        Xdist = targetX - CurrentXPos;
        Ydist = targetY - CurrentYPos;

        if (StartingHeading <= 0) {
            ConvertedHeading = (0 - StartingHeading);
        }else {
            ConvertedHeading = (360 - StartingHeading);
        }

        RRXdist = Xdist*Math.cos(Math.toRadians(360-ConvertedHeading)) - Ydist*Math.sin(Math.toRadians(360-ConvertedHeading));
        RRYdist = Xdist*Math.sin(Math.toRadians(360-ConvertedHeading)) + Ydist*Math.cos(Math.toRadians(360-ConvertedHeading));

        Vertical = drivePID.calculate(RRYdist);
        Horizontal = strafePID.calculate(RRXdist);
        Pivot = PivotPID.calculate(gyro.angle() - targetRot);

        drive.RF.setPower(-Pivot + (Vertical - Horizontal));
        drive.RB.setPower(-Pivot + (Vertical + Horizontal));
        drive.LF.setPower(Pivot + (Vertical + Horizontal));
        drive.LB.setPower(Pivot + (Vertical - Horizontal));

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
}
