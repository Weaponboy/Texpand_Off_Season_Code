package org.firstinspires.ftc.teamcode.Auto.Test_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Piplines.Old.OpenCVPipelinetest;

@Autonomous
@Disabled
public class Test_Cone_Detection extends LinearOpMode {

    Drivetrain drive = new Drivetrain();

    OpenCVPipelinetest open = new OpenCVPipelinetest();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        open.runOpMode();

        open.Get_Pos_1();
        open.Get_Pos_2();
        open.Get_Pos_3();

        if(open.Pos_1){

            drive.DriveDistance(59,0.75);
            drive.StrafeDistance_Left(50,0.75);

        }else if(open.Pos_2){

            drive.DriveDistance(50, .75);

        }else if(open.Pos_3){

            drive.DriveDistance(50,0.75);
            drive.StrafeDistance(50,0.75);

        }
    }
}
