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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

/**
 * This sample shows how to use dead wheels with external encoders
 * paired with motors that don't require encoders.
 * In this sample, we will use the drive motors' encoder
 * ports as they are not needed due to not using the drive encoders.
 * The external encoders we are using are REV through-bore.
 */
@TeleOp
public class DeadWheelsSample extends LinearOpMode {

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

    Drivetrain drive = new Drivetrain();

    private MotorEx LF, RF, LB, RB;
    private MecanumDrive driveTrain;
    private Encoder leftOdometer, rightOdometer, centerOdometer;
    private HolonomicOdometry odometry;

    @Override
    public void runOpMode() throws InterruptedException {

        LF = new MotorEx(hardwareMap, "LF");
        LB = new MotorEx(hardwareMap, "LB");
        RF = new MotorEx(hardwareMap, "RF");
        RB = new MotorEx(hardwareMap, "RB");

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

        drive.init(hardwareMap);

        waitForStart();

        TurnOdometry(90,0.4);

        while (opModeIsActive() && !isStopRequested()) {
            // control loop
            odometry.updatePose(); // update the position

            telemetry.addData("Robot Position", odometry.getPose());
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