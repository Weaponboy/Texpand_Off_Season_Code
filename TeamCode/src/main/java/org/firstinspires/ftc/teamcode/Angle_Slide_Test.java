package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Angle_Slide_Test extends OpMode {
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;
    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;
    public DcMotor Angle_slide = null;
    public Servo Intake = null;
    public Servo Intake_Angle = null;
    private double vertical;
    private double horizontal;
    private double pivot;
    private double AngleOfIntake = 0;
    private double Intakepos = 0;



    @Override
    public void loop() {
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(-pivot + (vertical - horizontal));
        RB.setPower(-pivot + (vertical + horizontal));
        LF.setPower(pivot + (vertical + horizontal));
        LB.setPower(pivot + (vertical - horizontal));

        if(gamepad1.dpad_left){
            Angle_slide.setPower(0.25);
        }else if(gamepad1.dpad_right){
            Angle_slide.setPower(-0.25);
        }else{
            Angle_slide.setPower(0);
        }

        if(gamepad1.dpad_up){
            Right_Slide.setPower(0.25);
            Left_Slide.setPower(0.25);
        }else if(gamepad1.dpad_down){
            Right_Slide.setPower(-0.25);
            Left_Slide.setPower(-0.25);
        }else{
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }

        if(gamepad1.right_bumper){
            AngleOfIntake = AngleOfIntake - 0.03;
            Intake_Angle.setPosition(AngleOfIntake);
            telemetry.addData("Intake_Angle", AngleOfIntake);
        }else if(gamepad1.left_bumper){
            AngleOfIntake = AngleOfIntake + 0.03;
            Intake_Angle.setPosition(AngleOfIntake);
            telemetry.addData("Intake_Angle", AngleOfIntake);
        }
        telemetry.update();
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        if(gamepad1.right_trigger>0 || gamepad1.right_trigger<0){
            Intake.setPosition(1);

        }else if(gamepad1.left_trigger>0 || gamepad1.left_trigger<0){
            Intake.setPosition(0);

        }
        telemetry.update();
        Intake.setPosition(-1);

        try {
            Thread.sleep(75);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


    }

    @Override
    public void init() {
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");
        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");
        Angle_slide = hardwareMap.get(DcMotor.class, "Slide angle motor");
        Intake = hardwareMap.get(Servo.class,"Intake");
        Intake_Angle = hardwareMap.get(Servo.class,"Intake Angle");
        Intake.setDirection(Servo.Direction.FORWARD);
        Intake_Angle.setDirection(Servo.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Angle_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }
}
