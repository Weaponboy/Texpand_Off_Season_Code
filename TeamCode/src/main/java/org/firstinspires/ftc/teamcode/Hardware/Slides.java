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
    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();

    public ColorSensor colour = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    private boolean lowering = false;
    private boolean conefound = false;
    private boolean extending = false;


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

    public void Full_Cycle_To_High_Pole(){

        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 150;

        while(!conefound && Extend.getCurrentPosition() > -1900){
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            conefound = colour.blue() > 150;
            Extend.setPower(-1);
            try {
                Thread.sleep(10);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }

        Extend.setPower(0);
        if(conefound) {
            if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                Right_Slide.setPower(0);
                Left_Slide.setPower(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lowering = false;
            }else if(lowering){
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
            }
            top.Top_Pivot.setPosition(1);
            top.Top_Gripper.setPosition(0.45);
            bottom.Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
            if(bottom.Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }
                    bottom.Base_Gripper.setPosition(0);
                    bottom.Destacker.setPosition(0.2);
                    bottom.Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0);
                if(bottom.Base_Pivot.getPosition() > 0.9) {
                    try {
                        Thread.sleep(70);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }
                    top.Top_Gripper.setPosition(0);
                    if (top.Top_Gripper.getPosition() == 0) {
                        bottom.Base_Gripper.setPosition(0.45);
                        try {
                            Thread.sleep(50);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }
                        if (bottom.Base_Gripper.getPosition() <= 0.45) {
                            top.Top_Pivot.setPosition(0.4);
                            if (top.Top_Pivot.getPosition() <= 0.35) {
                                bottom.Base_Pivot.setPosition(0.35);
                            }
                        }

                    }
                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else{
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
        }

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


        top.Top_Gripper.setPosition(0.45);
        if(top.Top_Gripper.getPosition() == 0.45) {
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            top.Top_Pivot.setPosition(1);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }

    }


    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        colour = hardwareMap.get(ColorSensor.class, "colour");

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
