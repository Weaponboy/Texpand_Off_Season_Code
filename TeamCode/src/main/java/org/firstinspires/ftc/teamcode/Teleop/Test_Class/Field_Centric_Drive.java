package org.firstinspires.ftc.teamcode.Teleop.Test_Class;

import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Odometry.PIDMovement.MovePIDTuning.strafeP;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.wolfpackmachina.bettersensors.Sensors.Gyro;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Field_Centric_Drive extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    public Orientation yawAngle;
    public BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    double RRXdist = 0;
    double RRYdist = 0;

    private ElapsedTime runtime = new ElapsedTime();

    private double vertical;
    private double horizontal;
    private double pivot;

    private double slow1 = 0.4;


    @Override
    public void loop() {

        slow1 = (gamepad1.left_trigger * 0.6) + 0.4;

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        Orientation bothead = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double botHeading = bothead.firstAngle;

        //CONVERT TARGET TO ROBOT RELATIVE TARGET
        RRXdist = vertical * Math.cos(-botHeading) - horizontal * Math.sin(-botHeading);

        RRYdist = vertical * Math.sin(-botHeading) + horizontal * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(RRYdist) + Math.abs(RRXdist) + Math.abs(pivot), 1);


        RF.setPower((slow1*1.4)*(-pivot + (vertical - horizontal)) / denominator);
        RB.setPower(slow1*(-pivot + (vertical + horizontal)) / denominator);
        LF.setPower((slow1*1.4)*(pivot + (vertical + horizontal)) / denominator);
        LB.setPower(slow1*(pivot + (vertical - horizontal)) / denominator);



        telemetry.addData("RF Power:", RF.getPower());
        telemetry.addData("RB Power:", RB.getPower());
        telemetry.addData("LF Power:", LF.getPower());
        telemetry.addData("LB Power:", LB.getPower());
        telemetry.update();

    }

    //init
    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }


}