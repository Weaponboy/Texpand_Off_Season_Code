package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Slides {

    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;
    public DcMotor Extend = null;

    Top_gripper top = new Top_gripper();

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    private boolean lowering = false;


    public Slides() { }

    public void Up (){
        Right_Slide.setPower(1);
        Left_Slide.setPower(1);
    }

    public void Down (){
        Right_Slide.setPower(-0.9);
        Left_Slide.setPower(-0.9);
    }

    public void Off (){
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);
    }

    public void Top_pole (){
        Right_Slide.setTargetPosition(1800);
        Left_Slide.setTargetPosition(1800);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        top.Top_Pivot.setPosition(0.3);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
