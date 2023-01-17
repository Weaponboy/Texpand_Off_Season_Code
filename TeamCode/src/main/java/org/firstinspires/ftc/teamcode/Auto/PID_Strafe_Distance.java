package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
@Disabled
public class PID_Strafe_Distance {
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


    public PID_Strafe_Distance() { }

    public int getTicks() {
        return ticks;
    }

    public void TurnDegrees(int degrees){
        ticks = (int) (ticksperdegree*degrees);
        ResetEncoders();

        RF.setTargetPosition(-ticks);
        LF.setTargetPosition(ticks);
        RB.setTargetPosition(-ticks);
        LB.setTargetPosition(ticks);

        RF.setPower(0.75);
        LF.setPower(0.75);
        RB.setPower(0.75);
        LB.setPower(0.75);

        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RF.isBusy()){
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
        }

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

    }

    public void DriveDistance(double distance, double speed) {
        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 538;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        ticks = (int)(distance * ticksPerRevolution / wheelCircumference);

        // Calculate the correction factor
        double correctionFactor = 1;

        // Apply the correction factor to the calculated number of encoder ticks
        ticks = (int)(ticks * (correctionFactor));

        // Set the target position for each motor
        RF.setTargetPosition(ticks);
        RB.setTargetPosition(ticks);
        LF.setTargetPosition(ticks);
        LB.setTargetPosition(ticks);

        // Set the motors to run to the target position
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the power of each motor
        RF.setPower(speed);
        RB.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);

        // Wait for the motors to reach their target positions
        while (RF.isBusy() && RB.isBusy() && LF.isBusy() && LB.isBusy()) {
            // Do nothing
        }
        // Reset the motors to the run without encoders mode
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Reverse the motors to stop the robot sooner
        RF.setPower(-speed);
        RB.setPower(-speed);
        LF.setPower(-speed);
        LB.setPower(-speed);
        try {
            Thread.sleep(50);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);


    }

    public void StrafeDistance(double Strafe_cm, double power) {

        executor = Executors.newFixedThreadPool(4);

        // Create a shared counter that is incremented by the threads
        counter = new AtomicInteger();

        runtime.reset();

        double target_Pos = 0;

        double target_15 = 0;
        double target_30 = 0;
        double target_80 = 0;

        ResetEncoders();
        // Constants for the encoder counts per revolution and gear ratio
        final int ENCODER_COUNTS_PER_REVOLUTION = 538;
        final double GEAR_RATIO = 1.0;

        // Calculate the number of ticks per revolution
        int ticksPerRevolution = (int)(ENCODER_COUNTS_PER_REVOLUTION / GEAR_RATIO);

        // Calculate the circumference of the wheel
        double wheelCircumference = Math.PI * 2 * (9.6 / 2.0);

        // Calculate the number of encoder ticks required to travel the given distance
        ticks = Math.toIntExact((long) (Strafe_cm * ticksPerRevolution / wheelCircumference));

        target_Pos = ticks;
        target_15 = 0.15*ticks;
        target_30 = 0.3*ticks;
        target_80 = 0.8*ticks;

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

        double finalTarget_Pos = target_Pos;

        double finalTarget_1 = target_15;
        double finalTarget_2 = target_30;
        double finalTarget_3 = target_80;

        executor.submit(() -> {

            while (RF.getCurrentPosition() < finalTarget_1){
                RF.setPower(0.4*-power);
            }

            while (RF.getCurrentPosition() > finalTarget_1 && RF.getCurrentPosition() < finalTarget_2){
                RF.setPower(-power);
            }

            while (RF.getCurrentPosition() > finalTarget_2 && RF.getCurrentPosition() < finalTarget_3){
                RF.setPower(0.4*-power);
            }
            RF.setPower(0);

        });

        executor.submit(() -> {

            while (RB.getCurrentPosition() != finalTarget_Pos){
                RB.setPower(power);
            }
            RB.setPower(0);

        });

        executor.submit(() -> {

            while (LF.getCurrentPosition() < finalTarget_1){
                LF.setPower(0.4*power);
            }

            while (LF.getCurrentPosition() > finalTarget_1 && LF.getCurrentPosition() < finalTarget_2){
                LF.setPower(power);
            }

            while (LF.getCurrentPosition() > finalTarget_2 && LF.getCurrentPosition() < finalTarget_3){
                LF.setPower(0.4*power);
            }
            LF.setPower(0);

        });

        executor.submit(() -> {

            while (LB.getCurrentPosition() != -finalTarget_Pos){
                LB.setPower(-power);
            }
            LB.setPower(0);
        });

        // Reset the motors to the run without encoders mode
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop the motors
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        LB.setPower(0);

        executor.shutdown();
        try {
            executor.awaitTermination(1, TimeUnit.MINUTES);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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
