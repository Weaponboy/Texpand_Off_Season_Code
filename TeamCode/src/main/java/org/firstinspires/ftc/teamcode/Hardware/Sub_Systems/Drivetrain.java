package org.firstinspires.ftc.teamcode.Hardware.Sub_Systems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odometry.DeadWheelsSample;

public class Drivetrain {

    private int ticks = 0;
    static double ticksperdegree = 11.111;
    static double circumference = 30.144;

    public DistanceSensor sensorRange;

    public double Distance_000 = 0;
    public double Distance_00 = 0;
    public double Distance_0 = 0;
    public double Distance_1 = 0;
    public double Distance_2 = 0;
    public double Distance_3 = 0;
    public double Distance_4 = 0;
    public double Distance_5 = 0;
    public double Distance_6 = 0;
    public double Distance_7 = 0;

    public double Av_Distance_1 = 0;
    public double Av_Distance_2 = 0;
    public double Av_Distance_3 = 0;
    public double Av_Distance_4 = 0;
    public double Av_Distance_5 = 0;
    public double Av_Distance_6 = 0;
    public double Av_Distance_7 = 0;

    public double Distance_latest = 0;
    
    Orientation yawAngle;

    static final double     HEADING_THRESHOLD       = 1.0 ;

    static final double     P_TURN_GAIN            = 0.02;
    static final double     P_DRIVE_GAIN           = 0.03;

    private BNO055IMU imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    private double  Current_Heading = 0;

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();


    public Drivetrain() { }

    public void stopMotors(){
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }

    public int getTicks() {
        return ticks;
    }

    public void TurnDegreesLeft(double degrees){

        ticks = (int) (ticksperdegree*degrees);
        ResetEncoders();

        RF.setTargetPosition(ticks);
        LF.setTargetPosition(-ticks);
        RB.setTargetPosition(ticks);
        LB.setTargetPosition(-ticks);

        RF.setPower(0.7);
        LF.setPower(-0.7);
        RB.setPower(0.7);
        LB.setPower(-0.7);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.getCurrentPosition() < ticks - 5 || RB.getCurrentPosition() < ticks - 5 || LF.getCurrentPosition() > ticks + 5 || LB.getCurrentPosition() > ticks + 5) {

            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() < -ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() < -ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void TurnDegrees(double degrees){

        ticks = (int) (ticksperdegree*degrees);
        ResetEncoders();

        RF.setTargetPosition(-ticks);
        LF.setTargetPosition(ticks);
        RB.setTargetPosition(-ticks);
        LB.setTargetPosition(ticks);

        RF.setPower(0.7);
        LF.setPower(-0.7);
        RB.setPower(0.7);
        LB.setPower(-0.7);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.getCurrentPosition() > ticks + 5 || RB.getCurrentPosition() > ticks + 5 || LF.getCurrentPosition() < ticks - 5 || LB.getCurrentPosition() < ticks - 5) {

            if(RB.getCurrentPosition() < -ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() < -ticks){
                RF.setPower(0);
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void TurnToHeading(double Heading){
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Current_Heading = yawAngle.firstAngle;
        while (Current_Heading < (Heading - 0.5) || Current_Heading > (Heading + 0.5)) {
            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Current_Heading = yawAngle.firstAngle;

            if (Current_Heading > (Heading + 0.25)) {
                LF.setPower(0.25);
                LB.setPower(0.25);
                RB.setPower(-0.25);
                RF.setPower(-0.25);
            } else if (Current_Heading < (Heading - 0.25)) {
                LF.setPower(-0.25);
                LB.setPower(-0.25);
                RB.setPower(0.25);
                RF.setPower(0.25);
            } else {
                RB.setPower(0);
                RF.setPower(0);
                LF.setPower(0);
                LB.setPower(0);
            }
        }
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void DriveDistanceLongReverse(double distance, double speed) {
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Start_angle = yawAngle.firstAngle;
        double Ramp_Down_point = 0;

        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (distance * ticksPerRevolution / wheelCircumference));


        // Set the target position for each motor
        RF.setTargetPosition(-ticks);
        RB.setTargetPosition(-ticks);
        LF.setTargetPosition(-ticks);
        LB.setTargetPosition(-ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set the power of each motor
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() > ((-ticks) + 20) || RB.getCurrentPosition() > ((-ticks) + 20) || LF.getCurrentPosition() > ((-ticks) + 20) || LB.getCurrentPosition() > ((-ticks) + 20) ) {

            RF.setPower(speed);
            RB.setPower(speed);
            LF.setPower(speed);
            LB.setPower(speed);


        }
        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void Drive() {

        DriveDistanceLong(130, 0.6);

        TurnToHeading(-89);

        DriveDistanceLong(25.5, 0.4);

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

        ResetEncoders();

        StrafeDistance_Left(10, 0.6);

        TurnToHeading(-96);

        Distance_1 = sensorRange.getDistance(DistanceUnit.MM);

        Distance_2 = sensorRange.getDistance(DistanceUnit.MM);

        while (Distance_2 - Distance_1 < 35){

            StrafeDistance_Left(2.5, 0.4);

            Distance_2 = sensorRange.getDistance(DistanceUnit.MM);

        }

    }

    public void DriveDistanceLong(double distance, double speed) {
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Start_angle = yawAngle.firstAngle;
        double Ramp_Down_point = 0;

        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (distance * ticksPerRevolution / wheelCircumference));

        // Set the target position for each motor
        RF.setTargetPosition(ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp_Down_point = 0.8*ticks;

        // Set the power of each motor
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() < ticks - 20 || RB.getCurrentPosition() < ticks - 20 || LF.getCurrentPosition() < ticks - 20 || LB.getCurrentPosition() < ticks - 20) {
            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Current_Heading = yawAngle.firstAngle;
            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }else if(RB.getCurrentPosition() >= Ramp_Down_point){
                RB.setPower(speed- 0.2);
            }

            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }else if(LF.getCurrentPosition() >= Ramp_Down_point){
                LF.setPower(speed- 0.2);
            }

            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }else if(LB.getCurrentPosition() >= Ramp_Down_point){
                LB.setPower(speed- 0.2);
            }

            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }else if(RF.getCurrentPosition() >= Ramp_Down_point){
                RF.setPower(speed- 0.2);
            }

            if (Current_Heading > (Start_angle + 3)) {
                LF.setPower(speed + 0.05);
                LB.setPower(speed + 0.05);
                RB.setPower(speed - 0.05);
                RF.setPower(speed - 0.05);
            } else if (Current_Heading < (Start_angle - 3)) {
                RB.setPower(speed + 0.05);
                RF.setPower(speed + 0.05);
                LF.setPower(speed - 0.05);
                LB.setPower(speed - 0.05);
            } else {
                RB.setPower(speed);
                RF.setPower(speed);
                LF.setPower(speed);
                LB.setPower(speed);
            }

        }
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Current_Heading = yawAngle.firstAngle;
        while (Current_Heading < (Start_angle - 3) || Current_Heading > (Start_angle + 3)) {
            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            Current_Heading = yawAngle.firstAngle;

            if (Current_Heading > (Start_angle + 1)) {
                LF.setPower(0.22);
                LB.setPower(0.22);
                RB.setPower(-0.22);
                RF.setPower(-0.22);
            } else if (Current_Heading < (Start_angle - 1)) {
                RB.setPower(0.22);
                RF.setPower(0.22);
                LF.setPower(-0.22);
                LB.setPower(-0.22);
            } else {
                RB.setPower(0);
                RF.setPower(0);
                LF.setPower(0);
                LB.setPower(0);
            }
        }

        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void DriveDistance(double distance, double speed) {
        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (distance * ticksPerRevolution / wheelCircumference));


        // Set the target position for each motor
        RF.setTargetPosition(ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of each motor
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() < ticks - 20 || RB.getCurrentPosition() < ticks - 20 || LF.getCurrentPosition() < ticks - 20 || LB.getCurrentPosition() < ticks - 20) {

            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double firstAngle = yawAngle.secondAngle;


            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }
        }

        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);


    }

    public void StrafeDistance(double Strafe_cm, double power) {
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Start_angle = yawAngle.firstAngle;
        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (1.2*Strafe_cm * ticksPerRevolution / wheelCircumference));

        // Set the target position for each motor
        RF.setTargetPosition(-ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(-ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of each motor
        RF.setPower(power);
        RB.setPower(power);
        LF.setPower(power);
        LB.setPower(power);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() > (-ticks) + 20 || RB.getCurrentPosition() < ticks - 30 || LF.getCurrentPosition() < ticks - 20 || LB.getCurrentPosition() > (-ticks) + 30) {
//            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            Current_Heading = yawAngle.firstAngle;

            if(RB.getCurrentPosition() > ticks){
                RB.setPower(0);
            }
            if(LF.getCurrentPosition() > ticks){
                LF.setPower(0);
            }
            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }

//            if (Current_Heading > (Start_angle + 1)) {
//                RF.setPower(power + 0.1);
//                LF.setPower(power + 0.1);
//            } else if (Current_Heading < (Start_angle - 1)) {
//                LB.setPower(power + 0.1);
//                RB.setPower(power + 0.1);
//            } else {
//                RB.setPower(power);
//                RF.setPower(power);
//                LF.setPower(power);
//                LB.setPower(power);
//            }
        }

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

//        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        Current_Heading = yawAngle.firstAngle;
//
//        while (Current_Heading < (Start_angle - 3) || Current_Heading > (Start_angle + 3)) {
//            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            Current_Heading = yawAngle.firstAngle;
//
//            if (Current_Heading > (Start_angle + 1)) {
//                LF.setPower(0.22);
//                RF.setPower(-0.22);
//            } else if (Current_Heading < (Start_angle - 1)) {
//                RB.setPower(0.22);
//                LB.setPower(-0.22);
//            } else {
//                RB.setPower(0);
//                RF.setPower(0);
//                LF.setPower(0);
//                LB.setPower(0);
//            }
//        }

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void StrafeOdometryRight(double TargetPos, double power) {

        double CurrentPos = 0;

        double error = 2.5;

        double Distance_to_travel = 0;

        Distance_to_travel = TargetPos - CurrentPos - error;


        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Distance_to_travel > 0){

            while (CurrentPos < TargetPos){

                RF.setPower(power);
                RB.setPower(-power);
                LF.setPower(-power);
                LB.setPower(power);
            }

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);


        }else if (Distance_to_travel < 0) {

            while (CurrentPos < TargetPos){
                RF.setPower(-power);
                RB.setPower(power);
                LF.setPower(power);
                LB.setPower(-power);
            }

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

        }





        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void StrafeDistance_Left(double Strafe_cm, double power) {
        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double Start_angle = yawAngle.firstAngle;
        double Ramp_Down_point = 0;

        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        int ticks = Math.toIntExact((long) (1.2*Strafe_cm * ticksPerRevolution / wheelCircumference));


        // Set the target position for each motor
        RF.setTargetPosition(ticks);
        RB.setTargetPosition(-ticks);
        LF.setTargetPosition(-ticks);
        LB.setTargetPosition(ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp_Down_point = 0.8*ticks;

        // Set the power of each motor
        RF.setPower(power);
        RB.setPower(power);
        LF.setPower(power);
        LB.setPower(power);

        // Wait for the motors to reach their target positions
        while (RF.getCurrentPosition() < (ticks) - 30 || RB.getCurrentPosition() > (-ticks) + 20 || LF.getCurrentPosition() > (-ticks) + 30 || LB.getCurrentPosition() < (ticks) - 20) {

            if(RB.getCurrentPosition() < -ticks){
                RB.setPower(0);
            }
//            else if(RB.getCurrentPosition() >= Ramp_Down_point){
//                RB.setPower(0.2);
//            }

            if(LF.getCurrentPosition() < -ticks){
                LF.setPower(0);
            }
//            else if(LF.getCurrentPosition() >= Ramp_Down_point){
//                LF.setPower(0.2);
//            }

            if(LB.getCurrentPosition() > ticks){
                LB.setPower(0);
            }
//            else if(LB.getCurrentPosition() >= Ramp_Down_point){
//                LB.setPower(0.2);
//            }

            if(RF.getCurrentPosition() > ticks){
                RF.setPower(0);
            }
//            else if(RF.getCurrentPosition() >= Ramp_Down_point){
//                RF.setPower(0.2);
//            }

//            if (Current_Heading > (Start_angle + 1)) {
//                RF.setPower(power + 0.1);
//                LF.setPower(power + 0.1);
//            } else if (Current_Heading < (Start_angle - 1)) {
//                LB.setPower(power + 0.1);
//                RB.setPower(power + 0.1);
//            } else {
//                RB.setPower(power);
//                RF.setPower(power);
//                LF.setPower(power);
//                LB.setPower(power);
//            }
        }

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

//        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        Current_Heading = yawAngle.firstAngle;
//
//        while (Current_Heading < (Start_angle - 3) || Current_Heading > (Start_angle + 3)) {
//            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            Current_Heading = yawAngle.firstAngle;
//
//            if (Current_Heading > (Start_angle + 1)) {
//                LF.setPower(0.22);
//                RF.setPower(-0.22);
//
//            } else if (Current_Heading < (Start_angle - 1)) {
//
//                RB.setPower(0.22);
//                LB.setPower(-0.22);
//            } else {
//                RB.setPower(0);
//                RF.setPower(0);
//                LF.setPower(0);
//                LB.setPower(0);
//            }
//        }
        RB.setPower(0);
        RF.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void ResetEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void DriveEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void WithOutEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Hold_Pos() {

        double Ramp_Down_point = 0;

        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 510;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance

        // Wait for the motors to reach their target positions


            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double firstAngle = yawAngle.secondAngle;
            firstAngle = Current_Heading;

            if(Current_Heading > 5){
                RB.setPower(0.3);
                LF.setPower(0.3);
            }

            if(Current_Heading < -5){
                RF.setPower(0.3);
                LB.setPower(0.3);
            }



        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void init(HardwareMap hwMap) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";



        hardwareMap = hwMap;

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }

}
