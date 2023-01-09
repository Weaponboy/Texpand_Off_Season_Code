package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Vision.OpenCVPipelinetest;
import org.firstinspires.ftc.teamcode.Vision.Stack_Detection_Blue;
import org.firstinspires.ftc.teamcode.Vision.Stack_Detection_Red;

@Autonomous
public class Blue_Cycle_Auto_A2_Starting_Position extends LinearOpMode {

    //Vision for finding the cone stack
    Stack_Detection_Blue line = new Stack_Detection_Blue();

    //vision to find the parking position
    OpenCVPipelinetest sleeve = new OpenCVPipelinetest();

    //Drivetrain object
    Drivetrain drive = new Drivetrain();

    //Bottom gripper system object
    Bottom_Gripper_Assembly B_grip = new Bottom_Gripper_Assembly();

    //All slides object
    Slides slide = new Slides();

    //Top gripper and pivot object
    Top_gripper top = new Top_gripper();



    @Override
    public void runOpMode() throws InterruptedException {

        //init all hardware
        drive.init(hardwareMap);
        B_grip.init(hardwareMap);
        slide.init(hardwareMap);
        top.init(hardwareMap);

        waitForStart();

        sleeve.runOpMode();

        if(sleeve.Pos_1 = true){
            //Code for cycling and stopping in position 1 goes here
            drive.DriveDistance(119, .75);
            drive.TurnDegrees(105);
            line.runOpMode();
            B_grip.Base_Gripper.setPosition(0.45);
            top.Top_Gripper.setPosition(0);
            top.Top_Pivot.setPosition(0.4);
            slide.Top_pole();
            slide.return_Zero();
            slide.Destack_5();
            drive.TurnDegrees(-15);
            drive.StrafeDistance(15,0.75);
            drive.DriveDistance(-59,0.75);

        }else if(sleeve.Pos_2 = true){
            //Code for cycling and stopping in position 2 goes here
            drive.DriveDistance(119, .75);
            drive.TurnDegrees(105);
            line.runOpMode();
            B_grip.Base_Gripper.setPosition(0.45);
            top.Top_Gripper.setPosition(0);
            top.Top_Pivot.setPosition(0.4);
            slide.Top_pole();
            slide.return_Zero();
            slide.Destack_5();
            drive.TurnDegrees(-15);
            drive.StrafeDistance(15,0.75);


        }else if(sleeve.Pos_3 = true){
            //Code for cycling and stopping in position 3 goes here
            drive.DriveDistance(119, .75);
            drive.TurnDegrees(105);
            line.runOpMode();
            B_grip.Base_Gripper.setPosition(0.45);
            top.Top_Gripper.setPosition(0);
            top.Top_Pivot.setPosition(0.4);
            slide.Top_pole();
            slide.return_Zero();
            slide.Destack_5();
            drive.TurnDegrees(-15);
            drive.StrafeDistance(15,0.75);
            drive.DriveDistance(59,0.75);

        }
    }
}
