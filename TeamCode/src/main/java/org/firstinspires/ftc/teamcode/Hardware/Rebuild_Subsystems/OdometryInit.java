package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeP;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.wolfpackmachina.bettersensors.HardwareMapProvider;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

public class OdometryInit{


    @Config
    public static class MovePIDTuning{

        public static double driveP = 0.1;
        public static double driveD = 0.01;
        public static double driveF = 0;

        public static double strafeP = 0.1;
        public static double strafeD = 0.005;
        public static double strafeF = 0;

        public static double rotationP = 0.05;
        public static double rotationD = 0.005;
        public static double rotationF = 0;
    }

    public Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    public HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 36.32;

    public MotorEx LF, RF, LB, RB;

    Gyro gyro;

    PIDFController drivePID;
    PIDFController strafePID;
    PIDFController PivotPID;

    public static final double CENTER_WHEEL_OFFSET = -17;

    public static final double WHEEL_DIAMETER = 3.5;

    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MecanumDrive driveTrain;

    HardwareMap hardwareMap = null;


    public void init(HardwareMap hwMap){

        hardwareMap = hwMap;

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

        driveTrain = new MecanumDrive(RB, RF, LB, LF);

        leftOdometer = LF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdometer = RF.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdometer = LB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d(0)));

        odometry.update(0, 0, 0);

        odometry.updatePose();

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
