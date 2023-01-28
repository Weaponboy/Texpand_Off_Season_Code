package org.firstinspires.ftc.teamcode.Auto.Old_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
@Disabled
public class PID_Strafe_Distance_2 {
    private int ticks = 0;
    static double ticksperdegree = 1.493;
    static double circumference = 30.144;

    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    private ExecutorService executor;
    private AtomicInteger counter;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public PID_Strafe_Distance_2() { }


    public void StrafeDistance(double Strafe_cm, double power) {

        double target_Pos = 0;

        double RampUp = 0;
        double Full_Speed = 0;
        double RampDown = 0;

        double RF_pos = 0;
        double LF_pos = 0;
        double RB_pos = 0;
        double LB_pos = 0;

        double Fill = 0;
        double Fill2 = 0;
        double Fill3 = 0;

        double Fill_M = 0;
        double Fill2_M = 0;
        double Fill3_M = 0;

        double Small_Def = 0;
        double Large_Def = 0;

        double Greatest_Power_Change = 0;

        double Greatest_Def = 0;

        double Avr_pos = 0;

        ResetEncoders();

        final int ENCODER_COUNTS_PER_REVOLUTION = 538;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        ticks = Math.toIntExact((long) (Strafe_cm * ticksPerRevolution / wheelCircumference));

        target_Pos = ticks;
        RampUp = 0.1*ticks;
        Full_Speed = 0.15*ticks;
        RampDown = 0.9*ticks;

        // Set the target position for each motor
        RF.setTargetPosition(-ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(-ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RB.setPower(power);
        LB.setPower(-power);
        RF.setPower(0.6*-power);
        LF.setPower(0.6*power);

        double Test_1 = 0;
        double Test_2 = 0;
        double Test_3 = 0;


        while (Avr_pos < RampUp){

            RF_pos = RF.getCurrentPosition();
            LF_pos = LF.getCurrentPosition();
            LB_pos = LB.getCurrentPosition();
            RB_pos = RB.getCurrentPosition();

            Avr_pos = RF_pos + LF_pos + LB_pos + RB_pos / 4;

            Test_1 = RF_pos + LF_pos;

            //Greatest value
            Fill = Math.max(RF_pos, LF_pos);
            Fill2 = Math.max(RB_pos, LB_pos);
            Large_Def = Math.max(Fill2, Fill);

            //Lowest value
            Fill_M = Math.min(RF_pos, LF_pos);
            Fill2_M = Math.min(RB_pos, LB_pos);
            Small_Def = Math.min(Fill_M, Fill2_M);

            Greatest_Def = Large_Def - Small_Def;

//            Avr_pos / Greatest_Def * Greatest_Power_Change + 1;



        }

        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;
        //Driving motors config
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }

    public void ResetEncoders(){
        //stop and reset the driving encoders
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
