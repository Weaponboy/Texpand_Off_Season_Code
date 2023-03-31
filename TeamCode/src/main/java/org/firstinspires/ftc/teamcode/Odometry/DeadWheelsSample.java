package org.firstinspires.ftc.teamcode.Odometry;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */

@TeleOp
@Disabled
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
    Drivetrain drive = new Drivetrain();
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
        centerOdometer = RB.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdometer.setDirection(Motor.Direction.FORWARD);
        rightOdometer.setDirection(Motor.Direction.REVERSE);

        odometry = new HolonomicOdometry(
                leftOdometer::getDistance,
                rightOdometer::getDistance,
                centerOdometer::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


//        odometry.updatePose(new Pose2d(0,0, new Rotation2d(0,0)));

        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.update(0, 0, 0);

        drive.init(hardwareMap, 1);
        odometry.updatePose(new Pose2d(0, 0, new Rotation2d()));

        odometry.update(0, 0, 0);
        odometry.updatePose();
        telemetry.addData("Robot Position", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();

        waitForStart();

//        drive.TurnToHeading(-90,0.3);
//
//        for (int i = 0;  i < 300; i++){
//            telemetry.addData("Robot Position", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            telemetry.update();
//            try {
//                Thread.sleep(10);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//        }
//        drive.TurnToHeading(-90,0.3);
//
//        for (int i = 0;  i < 300; i++){
//            telemetry.addData("Robot Position", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            telemetry.update();
//            try {
//                Thread.sleep(10);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//        }
//        TurnOdometry(-90,0.3);
//
//        TurnOdometry(90, 0.5);

        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            odometry.updatePose();// update the position

            telemetry.addData("Left odo", leftOdometer.getPosition());
            telemetry.addData("Right odo", rightOdometer.getPosition());
            telemetry.addData("Robot Position", Math.toDegrees(getheading()));
            telemetry.addData("Robot Position Y", getYpos());
            telemetry.addData("Robot Position X", getXpos());
            telemetry.update();


        }
    }

    public void DriveToPos(double TargetXPos, double TargetYPos, double power){
        double CurrentXPos = 0;
        double CurrentYPos = 0;
        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double power_rampup = 0.05;
        double power_rampdown = power_rampup*1.2;
        double ramp_down_point = 0;
        double max_speed = 0;
        int loopcount =0;
        boolean refined = false;
        double Xdist = 0;
        double Ydist = 0;
        double RRXdist = 0;
        double RRYdist = 0;
        double Horizontal = 0;
        double Vertical = 0;
        double Rotation = 0;
        double ResultantDist = 0;
        double ConvertedHeading = 0;
        double Pivot = 0;

        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        do {

            //update position
            odometry.updatePose();

            //Get bot positions
            CurrentXPos = getYpos();
            CurrentYPos = getXpos();

            StartingHeading = Math.toDegrees(getheading());

            //get converted heading
            if (StartingHeading <= 0) {
                ConvertedHeading = (0 - StartingHeading);
            }else {
                ConvertedHeading = (360 - StartingHeading);
            }

            //set distance to travel error
            Xdist = TargetXPos - CurrentXPos;
            Ydist = TargetYPos - CurrentYPos;

            RRXdist = Xdist*Math.cos(Math.toRadians(360-ConvertedHeading)) - Ydist*Math.sin(Math.toRadians(360-ConvertedHeading));
            RRYdist = Xdist*Math.sin(Math.toRadians(360-ConvertedHeading)) + Ydist*Math.cos(Math.toRadians(360-ConvertedHeading));

            ResultantDist = Math.sqrt((Xdist*Xdist) + (Ydist*Ydist));

            Vertical = Ydist/ResultantDist;
            Horizontal = Xdist/ResultantDist;


            telemetry.addData("Xdist", Xdist);
            telemetry.addData("Ydist", Ydist);
            telemetry.addData("StartingHeading", StartingHeading);
            telemetry.addData("ConvertedHeading", ConvertedHeading);
            telemetry.addData("RRXdist", RRXdist);
            telemetry.addData("RRYdist", RRYdist);
            telemetry.addData("Vertical", Vertical);
            telemetry.addData("Horizontal", Horizontal);
            telemetry.update();

            drive.RF.setPower(power*1.3*(-Pivot + (Vertical - Horizontal)));
            drive.RB.setPower(power*(-Pivot + (Vertical + Horizontal)));
            drive.LF.setPower(power*1.3*(Pivot + (Vertical + Horizontal)));
            drive.LB.setPower(power*(Pivot + (Vertical - Horizontal)));

        }while ((Math.abs(Xdist) > 0.5) || (Math.abs(Ydist) > 0.5));

//            ramp_down_point = Math.abs(TargetXPos - CurrentXPos)*0.75;
//            telemetry.addData("StartHeading", StartingHeading);
//            telemetry.addData("CurrentXPos", CurrentXPos);
//            telemetry.addData("TargetXPos", TargetXPos);
//            telemetry.addData("Rampdownpoint", ramp_down_point);
//            telemetry.update();
//        try {
//            Thread.sleep(4000);
//        } catch (Exception e) {
//            System.out.println(e.getMessage());
//        }
//            while (Math.abs(TargetXPos - CurrentXPos) > 0.05) {
//
//                telemetry.addData("Robot Position", odometry.getPose());
//                telemetry.addData("TargetXPos", TargetXPos);
//                telemetry.update();
//
//                odometry.updatePose();
//                CurrentXPos = getXpos();
//                if (Math.abs(TargetXPos - CurrentXPos) < ramp_down_point) {
//                    if (power > 0.25) {
//                        power = power - power_rampdown;
//                    }
//                }else if (power<1) {
//                    power = power + power_rampup;
//                    loopcount = loopcount +1;
//                }
//                if (power > max_speed){
//                    max_speed = power;
//                }
//                if (CurrentXPos < TargetXPos) {
//
//                    drive.RF.setPower(power);
//                    drive.RB.setPower(power);
//                    drive.LF.setPower(power);
//                    drive.LB.setPower(power);
//
//                } else if (CurrentXPos > TargetXPos) {
//
//                    drive.RF.setPower(-power);
//                    drive.RB.setPower(-power);
//                    drive.LF.setPower(-power);
//                    drive.LB.setPower(-power);
//                }
//            }
//            try {
//                Thread.sleep(150);
//            } catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
//            power = 0.15;
//            odometry.updatePose();
//            CurrentXPos = getXpos();
//            while (Math.abs(TargetXPos - CurrentXPos) > 0.05) {
//
//                refined = true;
//                telemetry.addData("Robot Position", odometry.getPose());
//                telemetry.addData("TargetXPos", TargetXPos);
//                telemetry.update();
//
//                odometry.updatePose();
//                CurrentXPos = getXpos();
//                if (Math.abs(TargetXPos - CurrentXPos) < ramp_down_point) {
//                    if (power > 0.25) {
//                        power = power - power_rampdown;
//                    }
//                }else if (power<1) {
//                    power = power + power_rampup;
//                    loopcount = loopcount +1;
//                }
//                if (power > max_speed){
//                    max_speed = power;
//                }
//                if (CurrentXPos < TargetXPos) {
//
//                    drive.RF.setPower(power);
//                    drive.RB.setPower(power);
//                    drive.LF.setPower(power);
//                    drive.LB.setPower(power);
//
//                } else if (CurrentXPos > TargetXPos) {
//
//                    drive.RF.setPower(-power);
//                    drive.RB.setPower(-power);
//                    drive.LF.setPower(-power);
//                    drive.LB.setPower(-power);
//                }
//            }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        try {
            Thread.sleep(4000);
        } catch (Exception e) {
            System.out.println(e.getMessage());
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

    public void StrafeOdometry(double Distance, double power) {
        double CurrentXPos = 0;

        double error = 0.48;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double CurrentYPos = 0;

        double TargetXPos = 0;
        double TargetYPos = 0;
        double poweradj = 0.085;

        odometry.updatePose();

        CurrentXPos = getXpos();
        CurrentYPos = getYpos();
        StartingHeading = getheading();

        if (StartingHeading == 0 || Math.abs(StartingHeading) == 180) {
            TargetXPos = 0;
        } else {
            TargetXPos = CurrentXPos + (Distance * Math.sin(Math.abs(StartingHeading)));
        }
        if (Math.abs(StartingHeading) == 90) {
            TargetYPos = 0;
        } else {
            TargetYPos = CurrentYPos + (Distance * Math.cos(Math.abs(StartingHeading)));
        }

        telemetry.addData("Xdist", Math.abs(TargetXPos-CurrentXPos));
        telemetry.addData("Ydist", Math.abs(TargetYPos-CurrentYPos));

        telemetry.update();

        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(TargetXPos-CurrentXPos) > Math.abs(TargetYPos-CurrentYPos)) {
            //TARGETING XPos
            telemetry.addData("StartHeading", StartingHeading);
            telemetry.addData("CurrentXPos", CurrentXPos);
            telemetry.addData("TargetXPos", TargetXPos);
            telemetry.update();

            while (CurrentXPos < TargetXPos - 0.05 || CurrentXPos > TargetXPos + 0.05) {

                telemetry.addData("Robot Position", odometry.getPose());
                telemetry.addData("TargetXPos", TargetXPos);
                telemetry.update();

                odometry.updatePose();
                CurrentXPos = getXpos();
                if (Math.abs(TargetXPos - CurrentXPos) < 10) {
                    power = 0.25;
                } else if (Math.abs(TargetXPos - CurrentXPos) < 5) {
                    power = 0.15;
                } else if (Math.abs(TargetXPos - CurrentXPos) < 1) {
                    power = 0.1;
                }
                if (CurrentXPos < TargetXPos) {

                    drive.RF.setPower(power+poweradj);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power-poweradj);
                    drive.LB.setPower(power);

                } else if (CurrentXPos > TargetXPos) {

                    drive.RF.setPower(-power-poweradj);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power+poweradj);
                    drive.LB.setPower(-power);
                }
            }
        } else {
            //TARGETING YPos
            telemetry.addData("StartHeading", StartingHeading);
            telemetry.addData("CurrentYPos", CurrentYPos);
            telemetry.addData("TargetYPos", TargetYPos);
            telemetry.update();

            while (CurrentYPos < TargetYPos - 0.05 || CurrentYPos > TargetYPos + 0.05) {

                telemetry.addData("Robot Position", odometry.getPose());
                telemetry.addData("TargetYPos", TargetYPos);
                telemetry.update();

                odometry.updatePose();
                CurrentYPos = getYpos();
                if (Math.abs(TargetYPos - CurrentYPos) < 10) {
                    power = 0.25;
                } else if (Math.abs(TargetYPos - CurrentYPos) < 5) {
                    power = 0.15;
                } else if (Math.abs(TargetYPos - CurrentYPos) < 1) {
                    power = 0.1;
                }
                if (CurrentYPos < TargetYPos) {

                    drive.RF.setPower(power+poweradj);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power-poweradj);
                    drive.LB.setPower(power);

                } else if (CurrentYPos > TargetYPos) {

                    drive.RF.setPower(-power-poweradj);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power+poweradj);
                    drive.LB.setPower(-power);
                }
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

//        TurnOdometry(StartingHeading,0.25);

//        while (CurrentPos < Distance_to_travel - 1 + CurrentPosStarting || CurrentPos > Distance_to_travel + 1 + CurrentPosStarting) {
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

    }

    public void DriveOdometry(double Distance, double power){
        double CurrentXPos = 0;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        double CurrentYPos = 0;

        double TargetXPos = 0;
        double TargetYPos = 0;
        double power_rampup = 0.05;
        double power_rampdown = power_rampup*1.2;
        double ramp_down_point = 0;
        double max_speed = 0;
        int loopcount =0;
        boolean refined = false;

        odometry.updatePose();
        CurrentXPos = getXpos();
        CurrentYPos = getYpos();
        StartingHeading = getheading();

        TargetYPos = CurrentYPos + (Distance * Math.cos(Math.toRadians(90)-Math.abs(StartingHeading)));
        TargetXPos = CurrentXPos + (Distance * Math.sin(Math.toRadians(90)-Math.abs(StartingHeading)));

        telemetry.addData("Xdist", Math.abs(TargetXPos-CurrentXPos));
        telemetry.addData("Ydist", Math.abs(TargetYPos-CurrentYPos));

        telemetry.update();


        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(TargetXPos-CurrentXPos) > Math.abs(TargetYPos-CurrentYPos)) {
            //TARGETING XPos
            ramp_down_point = Math.abs(TargetXPos - CurrentXPos)*0.75;
            telemetry.addData("StartHeading", StartingHeading);
            telemetry.addData("CurrentXPos", CurrentXPos);
            telemetry.addData("TargetXPos", TargetXPos);
            telemetry.addData("Rampdownpoint", ramp_down_point);
            telemetry.update();
            try {
                Thread.sleep(4000);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            while (Math.abs(TargetXPos - CurrentXPos) > 0.05) {

                telemetry.addData("Robot Position", odometry.getPose());
                telemetry.addData("TargetXPos", TargetXPos);
                telemetry.update();

                odometry.updatePose();
                CurrentXPos = getXpos();
                if (Math.abs(TargetXPos - CurrentXPos) < ramp_down_point) {
                    if (power > 0.25) {
                        power = power - power_rampdown;
                    }
                }else if (power<1) {
                    power = power + power_rampup;
                    loopcount = loopcount +1;
                }
                if (power > max_speed){
                    max_speed = power;
                }
                if (CurrentXPos < TargetXPos) {

                    drive.RF.setPower(power);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power);
                    drive.LB.setPower(power);

                } else if (CurrentXPos > TargetXPos) {

                    drive.RF.setPower(-power);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power);
                    drive.LB.setPower(-power);
                }
            }
            try {
                Thread.sleep(150);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            power = 0.15;
            odometry.updatePose();
            CurrentXPos = getXpos();
            while (Math.abs(TargetXPos - CurrentXPos) > 0.05) {

                refined = true;
                telemetry.addData("Robot Position", odometry.getPose());
                telemetry.addData("TargetXPos", TargetXPos);
                telemetry.update();

                odometry.updatePose();
                CurrentXPos = getXpos();
                if (Math.abs(TargetXPos - CurrentXPos) < ramp_down_point) {
                    if (power > 0.25) {
                        power = power - power_rampdown;
                    }
                }else if (power<1) {
                    power = power + power_rampup;
                    loopcount = loopcount +1;
                }
                if (power > max_speed){
                    max_speed = power;
                }
                if (CurrentXPos < TargetXPos) {

                    drive.RF.setPower(power);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power);
                    drive.LB.setPower(power);

                } else if (CurrentXPos > TargetXPos) {

                    drive.RF.setPower(-power);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power);
                    drive.LB.setPower(-power);
                }
            }
        } else {
            //TARGETING YPos
            telemetry.addData("StartHeading", StartingHeading);
            telemetry.addData("CurrentYPos", CurrentYPos);
            telemetry.addData("TargetYPos", TargetYPos);
            telemetry.update();

            while (CurrentYPos < TargetYPos - 0.05 || CurrentYPos > TargetYPos + 0.05) {

                telemetry.addData("Robot Position", odometry.getPose());
                telemetry.addData("TargetYPos", TargetYPos);
                telemetry.update();

                odometry.updatePose();
                CurrentYPos = getYpos();
                if (Math.abs(TargetYPos - CurrentYPos) < 10) {
                    power = 0.25;
                } else if (Math.abs(TargetYPos - CurrentYPos) < 5) {
                    power = 0.15;
                } else if (Math.abs(TargetYPos - CurrentYPos) < 1) {
                    power = 0.1;
                }
                if (CurrentYPos < TargetYPos) {

                    drive.RF.setPower(power);
                    drive.RB.setPower(power);
                    drive.LF.setPower(power);
                    drive.LB.setPower(power);

                } else if (CurrentYPos > TargetYPos) {

                    drive.RF.setPower(-power);
                    drive.RB.setPower(-power);
                    drive.LF.setPower(-power);
                    drive.LB.setPower(-power);
                }
            }
        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        telemetry.addData("TargetXpos   ", TargetXPos);
        telemetry.addData("CurrentXpos", CurrentXPos);
        telemetry.addData("refined", refined);
        telemetry.update();
        try {
            Thread.sleep(4000);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
//        TurnOdometry(StartingHeading,0.25);

//        power = 0.09;
//
//        while(CurrentXPos < TargetXPos - 0.05|| CurrentXPos > TargetXPos + 0.05){
//
//            telemetry.addData("Robot Position", odometry.getPose());
//            telemetry.update();
//            odometry.updatePose();
//            CurrentXPos = getXpos();
//            if (CurrentXPos < TargetXPos){
//
//                drive.RF.setPower(power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(power);
//
//            }else if(CurrentXPos > TargetXPos){
//
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-power);
//            }
//        }
//        drive.RF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LF.setPower(0);
//        drive.LB.setPower(0);
//
//        power = 0.25;
//        odometry.updatePose();

        //turn
//        while (StartingHeading-0.1 > Math.toDegrees(getheading()) || StartingHeading+0.1 < Math.toDegrees(getheading())){
//
//            odometry.updatePose();
////Risky way of turning - depends on current heading being exactly equal to start heading
//            if (StartingHeading > Math.toDegrees(getheading())){
//                // turn right
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-0.3*power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(0.3*power);
//            }else if (StartingHeading < Math.toDegrees(getheading())){
//                // turn left
//                drive.RF.setPower(power);
//                drive.RB.setPower(0.3*power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-0.3*power);
//            }else {
//                drive.RF.setPower(0);
//                drive.RB.setPower(0);
//                drive.LF.setPower(0);
//                drive.LB.setPower(0);
//            }
//
//        }

//        power = 0.29;
//        odometry.updatePose();
//
//        while (StartingY - 0.1 > getYpos() || StartingY + 0.1 < getYpos()){
//
//            odometry.updatePose();
//
//            if (StartingY > getYpos()){
//                //strafe left
//                drive.RF.setPower(power);
//                drive.RB.setPower(-power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(power);
//
//            }else if (StartingY < getYpos()){
//                //strafe right
//                drive.RF.setPower(-power);
//                drive.RB.setPower(power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(-power);
//            }else {
//                drive.RF.setPower(0);
//                drive.RB.setPower(0);
//                drive.LF.setPower(0);
//                drive.LB.setPower(0);
//            }
//
//        }

//        power = 0.25;
//        odometry.updatePose();
//
//        while (StartingHeading-0.1 > Math.toDegrees(getheading()) || StartingHeading+0.1 < Math.toDegrees(getheading())){
//
//            odometry.updatePose();
//
//            if (StartingHeading > Math.toDegrees(getheading())){
//                drive.RF.setPower(-power);
//                drive.RB.setPower(-0.3*power);
//                drive.LF.setPower(power);
//                drive.LB.setPower(0.3*power);
//            }else if (StartingHeading < Math.toDegrees(getheading())){
//                drive.RF.setPower(power);
//                drive.RB.setPower(0.3*power);
//                drive.LF.setPower(-power);
//                drive.LB.setPower(-0.3*power);
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

    }

    public void TurnOdometry(double TargetHeading, double power){
        double CurrentPos = 0;

        double error = 0;

        double StartingHeading = 0;

        double CurrentPosStarting = 0;

        //double StartingY = 0;

        CurrentPos = getXpos();

        CurrentPosStarting = getYpos();

        StartingHeading = Math.toDegrees(getheading());

        double StartingY = getYpos();

        double StartingX = getXpos();

        // Set the motors to run to the target position
        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometry.updatePose();

        while (Math.toDegrees(getheading()) < TargetHeading -7 || Math.toDegrees(getheading()) > TargetHeading + 7){

            odometry.updatePose();

            if (Math.toDegrees(getheading()) < TargetHeading){
                //turn clockwise
                drive.RF.setPower(-1.4*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(1.4*power);
                drive.LB.setPower(power);
            }else if (Math.toDegrees(getheading()) > TargetHeading){
                //turn anti-clockwise
                drive.RF.setPower(1.4*power);
                drive.RB.setPower(power);
                drive.LF.setPower(-1.4*power);
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
        power = 0.14;

        odometry.updatePose();

        while (Math.toDegrees(getheading()) < TargetHeading - 0.25 || Math.toDegrees(getheading()) > TargetHeading + 0.25){

            odometry.updatePose();

            if (Math.toDegrees(getheading()) < TargetHeading){
                //turn clockwise
                drive.RF.setPower(-1.4*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(1.4*power);
                drive.LB.setPower(power);
            }else if (Math.toDegrees(getheading()) > TargetHeading){
                //turn anti-clockwise
                drive.RF.setPower(1.4*power);
                drive.RB.setPower(power);
                drive.LF.setPower(-1.4*power);
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

    public void coordinatesInCM(double XPositionTarget, double YPositionTarget, double HeadingPositionTarget){

        double CurrentX = getXpos();

        double CurrentY = getYpos();

        double CurrentHeading = getheading();

        //X is strafing
        double XDistanceToDrive = XPositionTarget - CurrentX;

        //Y is forward
        double YDistanceToDrive = YPositionTarget - CurrentY;

        //Turning
        double HeadingToTurnTo = HeadingPositionTarget - CurrentHeading;
        
        if (getheading() < 10 || getheading() > 350){

            DriveOdometry(XDistanceToDrive, 0.6);

            StrafeOdometry(YDistanceToDrive, 0.6);

            TurnOdometry(HeadingToTurnTo, 0.6);

        }else if (getheading() > 80 || getheading() < 100 ) {

            DriveOdometry(-YDistanceToDrive, 0.6);

            StrafeOdometry(XDistanceToDrive, 0.6);

            TurnOdometry(HeadingToTurnTo, 0.6);

        }else if (getheading() > 170 || getheading() < 190 ) {

            DriveOdometry(YDistanceToDrive, 0.6);

            StrafeOdometry(-XDistanceToDrive, 0.6);

            TurnOdometry(HeadingToTurnTo, 0.6);

        }else if (getheading() > 260 || getheading() < 280 ) {

            DriveOdometry(-XDistanceToDrive, 0.6);

            StrafeOdometry(-YDistanceToDrive, 0.6);

            TurnOdometry(HeadingToTurnTo, 0.6);

        }

        odometry.updatePose();

    }

    public void coordinatesInFieldMats(double matPositionTarget){

        //A
        if(matPositionTarget == 1){
            coordinatesInCM(6, 8, 0);
        }else if (matPositionTarget == 2) {
            coordinatesInCM(6, 68, 0);
        }else if (matPositionTarget == 3) {
            coordinatesInCM(6, 128, 0);
        }else if (matPositionTarget == 4) {
            coordinatesInCM(6, 188, 0);
        }else if (matPositionTarget == 5) {
            coordinatesInCM(6, 248, 0);
        }else if (matPositionTarget == 6) {
            coordinatesInCM(6, 308, 0);
        }
        //B
        else if (matPositionTarget == 7) {
            coordinatesInCM(66, 8, 0);
        }else if (matPositionTarget == 8) {
            coordinatesInCM(66, 68, 0);
        }else if (matPositionTarget == 9) {
            coordinatesInCM(66, 128, 0);
        }else if (matPositionTarget == 10) {
            coordinatesInCM(66, 188, 0);
        }else if (matPositionTarget == 11) {
            coordinatesInCM(66, 248, 0);
        }else if (matPositionTarget == 12) {
            coordinatesInCM(66, 308, 0);
        }
        //C
        else if (matPositionTarget == 13) {
            coordinatesInCM(126, 8, 0);
        }else if (matPositionTarget == 14) {
            coordinatesInCM(126, 68, 0);
        }else if (matPositionTarget == 15) {
            coordinatesInCM(126, 128, 0);
        }else if (matPositionTarget == 16) {
            coordinatesInCM(126, 188, 0);
        }else if (matPositionTarget == 17) {
            coordinatesInCM(126, 248, 0);
        }else if (matPositionTarget == 18) {
            coordinatesInCM(126, 308, 0);
        }
        //D
        else if (matPositionTarget == 19) {
            coordinatesInCM(186, 8, 0);
        }else if (matPositionTarget == 20) {
            coordinatesInCM(186, 68, 0);
        }else if (matPositionTarget == 21) {
            coordinatesInCM(186, 128, 0);
        }else if (matPositionTarget == 22) {
            coordinatesInCM(186, 188, 0);
        }else if (matPositionTarget == 23) {
            coordinatesInCM(186, 248, 0);
        }else if (matPositionTarget == 24) {
            coordinatesInCM(186, 308, 0);
        }
        //E
        else if (matPositionTarget == 25) {
            coordinatesInCM(246, 8, 0);
        }else if (matPositionTarget == 26) {
            coordinatesInCM(246, 68, 0);
        }else if (matPositionTarget == 27) {
            coordinatesInCM(246, 128, 0);
        }else if (matPositionTarget == 28) {
            coordinatesInCM(246, 188, 0);
        }else if (matPositionTarget == 29) {
            coordinatesInCM(246, 248, 0);
        }else if (matPositionTarget == 30) {
            coordinatesInCM(246, 308, 0);
        }
        //F
        else if (matPositionTarget == 31) {
            coordinatesInCM(306, 8, 0);
        }else if (matPositionTarget == 32) {
            coordinatesInCM(306, 68, 0);
        }else if (matPositionTarget == 33) {
            coordinatesInCM(306, 128, 0);
        }else if (matPositionTarget == 34) {
            coordinatesInCM(306, 188, 0);
        }else if (matPositionTarget == 35) {
            coordinatesInCM(306, 248, 0);
        }else if (matPositionTarget == 36) {
            coordinatesInCM(306, 308, 0);
        }
    }
}