package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;

@Autonomous
public class TestDriveFunctions extends LinearOpMode {
    Drivetrain TestTrain = new Drivetrain();
    @Override
    public void runOpMode() throws InterruptedException {

        TestTrain.init(hardwareMap);

        waitForStart();

        TestTrain.TurnDegrees(90);
        telemetry.addData("Target Ticks:", TestTrain.getTicks());
        telemetry.addData("motor ticks Right Back:", TestTrain.RB.getCurrentPosition());
        telemetry.addData("motor ticks Left Front:", TestTrain.LF.getCurrentPosition());
        telemetry.addData("motor ticks Right Front:", TestTrain.RF.getCurrentPosition());
        telemetry.addData("motor ticks Left Back:", TestTrain.LB.getCurrentPosition());
        telemetry.addData("RF Power:", TestTrain.RF.getPower());
        telemetry.addData("RB Power:", TestTrain.RB.getPower());
        telemetry.addData("LF Power:", TestTrain.LF.getPower());
        telemetry.addData("LB Power:", TestTrain.LB.getPower());
        telemetry.update();
        try {
            Thread.sleep(20000);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
    }
}
