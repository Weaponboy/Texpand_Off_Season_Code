package org.firstinspires.ftc.teamcode.Odometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

@Autonomous
public class ChatGPT_PID extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 36.2  ;

    //Telemetry for dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Center wheel offset is the distance between the
    // center of rotation of the robot and the center odometer.
    // This is to correct for the error that might occur when turning.
    // A negative offset means the odometer is closer to the back,
    // while a positive offset means it is closer to the front.
    public static final double CENTER_WHEEL_OFFSET = -13;

    public static final double WHEEL_DIAMETER = 3.5;

    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private double vertical;
    private double horizontal;
    private double pivot;
    private double Distance_to_travel;
    Drivetrain drive = new Drivetrain();
    public BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    private MotorEx LF, RF, LB, RB;
    private MecanumDrive driveTrain;
    private Motor.Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private static final double KP = 1.0;
    private static final double KI = 0.1;
    private static final double KD = 0.4;

    private double lastError = 0;
    private double integral = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //Telemetry for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new MecanumDrive(RB, RF, LB, LF);

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
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

        drive.init(hardwareMap, 1);

        odometry.updatePose();

        waitForStart();

        moveToPosition(20, 20, 0, 0.4);

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

    public void MecanumDriveBase(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
    }

    public void moveToPosition(double targetX, double targetY, double targetRotation, double power) {

        ElapsedTime runtime = new ElapsedTime();

        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (runtime.seconds() < 10) { // example loop condition

            odometry.updatePose();

            double currentX = getXpos(); // implement this method to get the robot's current x position
            double currentY = getYpos(); // implement this method to get the robot's current y position
            double currentRotation = getheading(); // implement this method to get the robot's current rotation

            double distance = Math.hypot(targetX - currentX, targetY - currentY);
            double angle = Math.atan2(targetY - currentY, targetX - currentX) - Math.PI / 4;
            double error = targetRotation - currentRotation;

            // calculate the proportional component of the control signal
            double proportional = KP * distance * Math.cos(angle);

            // calculate the integral component of the control signal
            integral += error * runtime.seconds();
            integral = Math.max(Math.min(integral, 1), -1); // limit the integral term to prevent overshoot
            double integralComponent = KI * integral;

            // calculate the derivative component of the control signal
            double derivative = (error - lastError) / runtime.seconds();
            double derivativeComponent = KD * derivative;

            // combine the proportional, integral, and derivative components to get the final control signal
            double powerX = proportional * Math.cos(angle) + integralComponent * Math.sin(angle) + derivativeComponent * Math.cos(angle);
            double powerY = proportional * Math.sin(angle) + integralComponent * Math.cos(angle) + derivativeComponent * Math.sin(angle);
            double powerRotation = KD * error;

            // set the motor powers based on the control signals
            drive.LF.setPower(power*(powerRotation + (powerY + powerX)));
            drive.RF.setPower(power*(-powerRotation + (powerY - powerX)));
            drive.LB.setPower(power*(powerRotation + (powerY - powerX)));
            drive.RB.setPower(power*(-powerRotation + (powerY + powerX)));

            odometry.updatePose();

            // update the last error for the next iteration
            lastError = error;


        }

        // stop the motors when the loop is done
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


}
