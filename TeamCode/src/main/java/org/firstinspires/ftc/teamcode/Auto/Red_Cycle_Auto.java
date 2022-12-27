package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.bottom_gripper;
import org.firstinspires.ftc.teamcode.Vision.Stack_Detection;

@Autonomous
public class Red_Cycle_Auto extends LinearOpMode {

    Stack_Detection line = new Stack_Detection();
    Threshold_Pipeline pipe = new Threshold_Pipeline();
    Drivetrain drive = new Drivetrain();
    bottom_gripper B_grip = new bottom_gripper();
    Slides slide = new Slides();
    Top_gripper top = new Top_gripper();



    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        B_grip.init(hardwareMap);
        slide.init(hardwareMap);
        top.init(hardwareMap);
        line.runOpMode();

        waitForStart();

        if(pipe.blueT == true){


        }else if(pipe.yellowT == true){


        }else if(pipe.redT == true){


        }
    }
}
