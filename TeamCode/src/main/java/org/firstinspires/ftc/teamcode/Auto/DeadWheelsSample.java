package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class DeadWheelsSample extends LinearOpMode {

    // The lateral distance between the left and right odometers
    // is called the trackwidth. This is very important for
    // determining angle for turning approximations
    public static final double TRACKWIDTH = 36.32;

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
    public BNO055IMU imu = null;      // Control/Expansion Hub IMU

    private MotorEx LF, RF, LB, RB;
    private MecanumDrive driveTrain;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

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

//        intakeLeft = new Motor(hardwareMap, "intake_left");
//        intakeRight = new Motor(hardwareMap, "intake_right");
//        liftLeft = new Motor(hardwareMap, "lift_left");
//        liftRight = new Motor(hardwareMap, "lift_right");

        // Here we set the distance per pulse of the odometers.
        // This is to keep the units consistent for the odometry.
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

        odometry.update(0, 0, 0);

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.updatePose();

        waitForStart();
        

        while (opModeIsActive() && !isStopRequested()) {

            // control loop
            odometry.updatePose();// update the position

            telemetry.addData("center odo", centerOdometer.getPosition());
            telemetry.addData("Left odo", leftOdometer.getPosition());
            telemetry.addData("Right odo", rightOdometer.getPosition());
            telemetry.addData("Robot Position", Math.toDegrees(getheading()));
            telemetry.addData("Robot Position Y", getYpos());
            telemetry.addData("Robot Position X", getXpos());
            telemetry.update();

        }
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