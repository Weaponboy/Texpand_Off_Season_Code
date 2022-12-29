package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Vision.OpenCVPipelinetest;
import org.firstinspires.ftc.teamcode.Vision.Stack_Detection;

@Autonomous
public class Blue_Cycle_Auto extends LinearOpMode {

    //Vision for finding the cone stack
    Stack_Detection line = new Stack_Detection();

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
        line.runOpMode();

        waitForStart();

        if(sleeve.Pos_1 = true){
            //Code for cycling and stopping in position 1 goes here

        }else if(sleeve.Pos_2 = true){
            //Code for cycling and stopping in position 2 goes here

        }else if(sleeve.Pos_3 = true){
            //Code for cycling and stopping in position 3 goes here

        }
    }
}
