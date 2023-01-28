package org.firstinspires.ftc.teamcode.Auto.Test_Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;

@Autonomous
public class TestDriveFunctions extends LinearOpMode {
    Drivetrain TestTrain = new Drivetrain();
    Orientation yawAngle;

    static final double HEADING_THRESHOLD = 1.0 ;

    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    public double count = 0;

    private BNO055IMU imu = null;      // Control/Expansion Hub IMU

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    private double  Current_Heading = 0;

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;


    @Override
    public void runOpMode() throws InterruptedException {

        TestTrain.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

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

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        waitForStart();

        TestTrain.DriveDistanceLong(130, 0.6);

        TestTrain.TurnToHeading(-87);

        TestTrain.DriveDistanceLong(25.5, 0.4);

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

        TestTrain.ResetEncoders();

        TestTrain.StrafeDistance_Left(10, 0.6);

        TestTrain.TurnToHeading(-96);

        TestTrain.Distance_1 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

        TestTrain.Distance_000 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

        TestTrain.Distance_00 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

        TestTrain.Distance_0 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_2 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_3 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_4 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_5 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_6 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);

            TestTrain.StrafeDistance_Left(3.5, 0.4);

            TestTrain.Distance_7 = TestTrain.sensorRange.getDistance(DistanceUnit.MM);




        TestTrain.Av_Distance_1 = (TestTrain.Distance_1 + TestTrain.Distance_0 + TestTrain.Distance_00 + TestTrain.Distance_000)/4;

        TestTrain.Av_Distance_2 = (TestTrain.Distance_2 + TestTrain.Distance_1 + TestTrain.Distance_0 + TestTrain.Distance_00)/4;

        TestTrain.Av_Distance_3 = (TestTrain.Distance_3 + TestTrain.Distance_2 + TestTrain.Distance_1 + TestTrain.Distance_0)/4;

        TestTrain.Av_Distance_4 = (TestTrain.Distance_4 + TestTrain.Distance_3 + TestTrain.Distance_2 + TestTrain.Distance_1)/4;

        TestTrain.Av_Distance_5 = (TestTrain.Distance_5 + TestTrain.Distance_4 + TestTrain.Distance_3 + TestTrain.Distance_2)/4;

        TestTrain.Av_Distance_6 = (TestTrain.Distance_6 + TestTrain.Distance_5 + TestTrain.Distance_4 + TestTrain.Distance_3)/4;

        TestTrain.Av_Distance_7 = (TestTrain.Distance_7 + TestTrain.Distance_6 + TestTrain.Distance_5 + TestTrain.Distance_4)/4;

        if ( TestTrain.Av_Distance_7 >  TestTrain.Av_Distance_6){
            TestTrain.StrafeDistance(3.4, 0.4);
        }
        if ( TestTrain.Av_Distance_6 >  TestTrain.Av_Distance_5){
            TestTrain.StrafeDistance(3.4, 0.4);
        }
        if ( TestTrain.Av_Distance_5 >  TestTrain.Av_Distance_4){
            TestTrain.StrafeDistance(3.4, 0.4);
        }
        if ( TestTrain.Av_Distance_4 >  TestTrain.Av_Distance_3){
            TestTrain.StrafeDistance(3.4, 0.4);
        }
        if ( TestTrain.Av_Distance_3 >  TestTrain.Av_Distance_2){
            TestTrain.StrafeDistance(3.4, 0.4);
        }
        if ( TestTrain.Av_Distance_2 >  TestTrain.Av_Distance_1){
            TestTrain.StrafeDistance(3.4, 0.4);
        }

        telemetry.addData("Dis 1", TestTrain.Distance_1);
        telemetry.addData("Dis 1", TestTrain.Distance_2);
        telemetry.addData("Dis 1", TestTrain.Distance_3);
        telemetry.addData("Dis 1", TestTrain.Distance_4);
        telemetry.addData("Dis 1", TestTrain.Distance_5);
        telemetry.addData("Dis 1", TestTrain.Distance_6);
        telemetry.addData("Dis 1", TestTrain.Distance_7);

        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_1);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_2);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_3);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_4);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_5);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_6);
        telemetry.addData("Dis 1 av", TestTrain.Av_Distance_7);
        telemetry.update();

            try {
                Thread.sleep(500);
            } catch (Exception e) {
                System.out.println(e.getMessage());

            }


        try {
            Thread.sleep(50000);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }


//        TestTrain.DriveDistanceLongReverse(60,0.5);
//        while(opModeIsActive()){
//            telemetry.addData("motor ticks", RF.getCurrentPosition());
//            telemetry.update();
//        }


//        yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double Start_angle = yawAngle.firstAngle;
//        while (opModeIsActive()){
//
//            telemetry.addData("Current Heading:",Current_Heading);
//            telemetry.addData("YawAngle:",yawAngle);
//            telemetry.update();
//            yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            Current_Heading = yawAngle.firstAngle;
//
//            if(Current_Heading > (Start_angle + 1)){
//                LF.setPower(0.15);
//                LB.setPower(0.15);
//                RB.setPower(-0.15);
//                RF.setPower(-0.15);
//                telemetry.addData("Turning:","Left");
//                telemetry.update();
//            }else if(Current_Heading < (Start_angle - 1)){
//                RB.setPower(0.15);
//                RF.setPower(0.15);
//                LF.setPower(-0.15);
//                LB.setPower(-0.15);
//                telemetry.addData("Turning:","Right");
//                telemetry.update();
//            }else{
//                RB.setPower(0);
//                RF.setPower(0);
//                LF.setPower(0);
//                LB.setPower(0);
//            }
//        }

//        TestTrain.DriveDistanceLong(136, 0.6);
//
//        TestTrain.TurnDegrees(90);
//
//        TestTrain.DriveDistanceLong(27, 0.6);
//
//        TestTrain.ResetEncoders();
//
//        TestTrain.StrafeDistance_Left(18, 0.7);
//
//        TestTrain.TurnDegrees(16);

    }
}

