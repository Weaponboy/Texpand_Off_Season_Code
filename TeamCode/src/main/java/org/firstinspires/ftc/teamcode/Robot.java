package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    //Create motors
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;
    public DcMotor intake = null;
    public DcMotor lift = null;
    public DcMotor carousel = null;
    //create servo
    public Servo bucket = null;
    public BNO055IMU imu;

    int slidePossition0 = 0;
    int slidePossition1 = 1000;
    int slidePossition2 = 1700;
    int slidePossition3 = 2550;
    int currentPossition = 0;
    boolean Intake = false;


    //additional variables
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();


    public Robot() { }

    public void Dumpbucket() throws InterruptedException{

        bucket.setPosition(0.500);
        try {
            Thread.sleep(1900);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        bucket.setPosition(0.003);

    }

    public void Toggleintake(int reverse) {

        if (!Intake) {
            Intake = true;
            intake.setPower(reverse);

        } else if (Intake) {
            Intake = false;
            intake.setPower(0);
        }
    }


    public void LiftSetPossition(int possition){


        if(possition == 0){
            currentPossition = slidePossition0;
        }else if(possition == 1){
            currentPossition = slidePossition1;
        }else if(possition == 2){
            currentPossition = slidePossition2;
        }else if(possition == 3){
            currentPossition = slidePossition3;
        }

        lift.setTargetPosition(currentPossition);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()){ }
        lift.setPower(0);

    }

    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;
        //Driving motors config
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //servo config
        bucket = hardwareMap.get(Servo.class,"bucket");

        //set up motor directions
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        //set motor mode
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set zero power behavior to stop
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motor power to zero
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        intake.setPower(0);
        lift.setPower(0);
        carousel.setPower(0);

        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU .class, "imu");
        imu.initialize(parameters);
    }

    public void ResetEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void setALLPower(double motorPower) {
        RF.setPower(motorPower);
        LF.setPower(motorPower);
        RB.setPower(motorPower);
        LB.setPower(motorPower);
    }

    public void setMotorPower(double RF_Power, double LF_Power, double RB_Power, double LB_Power) {
        RF.setPower(RF_Power);
        LF.setPower(LF_Power);
        RB.setPower(RB_Power);
        LB.setPower(LB_Power);

    }
    public void Sleep(long time) throws InterruptedException{
        Thread.sleep(time);
    }
}