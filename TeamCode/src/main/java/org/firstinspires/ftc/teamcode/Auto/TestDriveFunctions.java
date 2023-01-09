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
        TestTrain.StrafeDistance(30,0.5);
        TestTrain.StrafeDistance(-30,0.5);
    }
}
