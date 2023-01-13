package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Hardware.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Top_gripper;

@TeleOp
public class Test_Con extends OpMode {

    Drivetrain drive = new Drivetrain();
    Bottom_Gripper_Assembly B_grip = new Bottom_Gripper_Assembly();
    Slides slide = new Slides();
    Top_gripper top = new Top_gripper();

    public ColorSensor colour = null;

    @Override
    public void init() {
        drive.init(hardwareMap);
        B_grip.init(hardwareMap);
        slide.init(hardwareMap);
        top.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("Working", "Well");
        telemetry.update();
    }
}
