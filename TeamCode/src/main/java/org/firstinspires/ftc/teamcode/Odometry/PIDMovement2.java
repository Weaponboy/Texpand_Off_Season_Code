package org.firstinspires.ftc.teamcode.Odometry;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.targetRot;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.targetX;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.targetY;

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
import com.wolfpackmachina.bettersensors.HardwareMapProvider;
import com.wolfpackmachina.bettersensors.Sensor;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

@TeleOp
public class PIDMovement2 extends OpMode {

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

    public static final double TRACKWIDTH = 36.32;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static final double CENTER_WHEEL_OFFSET = -13;

    public static final double WHEEL_DIAMETER = 3.5;

    Drivetrain drive = new Drivetrain();

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MecanumDrive driveTrain;

    private MotorEx LF, RF, LB, RB;

    Gyro gyro;

    @Config
    public static class MovePIDTuning{

        public static double driveP = 0.05;
        public static double driveD = 0;
        public static double driveF = 0.4;


        public static double strafeP = 0.06;
        public static double strafeD = 0;
        public static double strafeF = 0.4;

        public static double rotationP = 0;
        public static double rotationD = 0;
        public static double rotationF = 0;

        public static double targetX, targetY = 10, targetRot = 0;

    }

    @Override
    public void init() {

        HardwareMapProvider.setMap(this);

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

        gyro = new Gyro("imu", 0);

        drive.init(hardwareMap);

        driveTrain = new MecanumDrive(RB, RF, LB, LF);

        leftOdometer = LF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = RF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = RB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.update(0, 0, 0);

        odometry.updatePose();

    }

    @Override
    public void loop() {

        //UPDATE ODOMETRY
        odometry.updatePose();

        //GET CURRENT X
        CurrentXPos = getYpos();

        //GET CURRENT Y
        CurrentYPos = getXpos();

        gyro.update();

        //GET START HEADING WITH GYRO
        StartingHeadinggyro = gyro.angle();

        //GET START HEADING WITH ODOMETRY
        StartingHeading = Math.toDegrees(getheading());

        //TELEMETRY FOR DASHBOARD
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //PID FOR DRIVING IN THE Y DIRECTION
        drivePID.setPIDF(driveP, 0, driveD, driveF);

        //PID FOR DRIVING IN THE X DIRECTION
        strafePID.setPIDF(strafeP, 0, strafeD, strafeF);

        //PID FOR TURNING
        PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

        //SET DISTANCE TO TRAVEL ERROR
        Xdist = targetX - CurrentXPos;
        Ydist = targetY - CurrentYPos;

        //CONVERT HEADING FOR TRIG CALCS
        if (StartingHeading <= 0) {
            ConvertedHeading = (0 - StartingHeading);
        }else {
            ConvertedHeading = (360 - StartingHeading);
        }

        //CONVERT TARGET TO ROBOT RELATIVE TARGET
        RRXdist = Xdist*Math.cos(Math.toRadians(360-ConvertedHeading)) + Ydist*Math.sin(Math.toRadians(360-ConvertedHeading));
        RRYdist = Xdist*Math.sin(Math.toRadians(360-ConvertedHeading)) - Ydist*Math.cos(Math.toRadians(360-ConvertedHeading));

        //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
        Vertical = drivePID.calculate(RRYdist);
        Horizontal = strafePID.calculate(RRXdist);
        Pivot = PivotPID.calculate(targetRot - StartingHeadinggyro);

        telemetry.addData("vertical power", Vertical);
        telemetry.addData("horizontal power", Horizontal);
        telemetry.addData("f pivot", rotationF);
        telemetry.addData("gyro heading", StartingHeadinggyro);
        telemetry.addData("odo heading", StartingHeading);
        telemetry.addData("converted heading", ConvertedHeading);
        telemetry.addData("X", getXpos());
        telemetry.addData("Y", getYpos());
        telemetry.update();

        //SET MOTOR POWER USING THE PID OUTPUT
        drive.RF.setPower(-Pivot + (Vertical - Horizontal));
        drive.RB.setPower((-Pivot*1.4) + (Vertical + (Horizontal*1.3)));
        drive.LF.setPower(Pivot + (Vertical + Horizontal));
        drive.LB.setPower((Pivot*1.4) + (Vertical - (Horizontal*1.3)));

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
