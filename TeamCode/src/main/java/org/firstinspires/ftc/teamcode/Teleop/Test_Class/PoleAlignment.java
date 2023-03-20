package org.firstinspires.ftc.teamcode.Teleop.Test_Class;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.openftc.apriltag.AprilTagDetection;

@TeleOp
public class PoleAlignment extends LinearOpMode {

    private DistanceSensor sensorRange;

    //Telemetry for dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    Drivetrain Drive = new Drivetrain();

    double aveDist = 0;
    double totaldist = 0;

    double Drive_F = 0;
    double Drive_R = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Drive.init(hardwareMap, 1);

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //Telemetry for dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();


        while(sensorRange.getDistance(DistanceUnit.MM) > 800) {
            RF.setPower(0.3);
            RB.setPower(-0.3);
            LF.setPower(-0.3);
            LB.setPower(0.3);

            telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.MM));
            telemetry.update();

        }

        Drive.StrafeDistance_Left(6,0.3);

        for (int i = 0; i < 10; i++){
            totaldist = totaldist + sensorRange.getDistance(DistanceUnit.MM);
        }
        aveDist = totaldist/10;

        if(aveDist > 390){
            Drive_F = (aveDist - 390)/10;
            Drive.DriveDistanceLongReverse(Drive_F, 0.2);
        }else if(aveDist < 390){
            Drive_R = (390 - aveDist)/10;
            Drive.DriveDistanceLong(Drive_R, 0.2);
        }

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

        telemetry.addData("Distance RAW:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("Distance AVE:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.update();

        try {
            Thread.sleep(7000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

    }


}
