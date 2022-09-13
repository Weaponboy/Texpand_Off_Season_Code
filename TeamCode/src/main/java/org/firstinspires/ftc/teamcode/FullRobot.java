package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp
public class FullRobot extends OpMode {

    Robot DriverBot = new Robot();


    private double vertical;
    private double horizontal;
    private double pivot;
    private int levelIndicator = 3;
    private double slow = 1;
    private int currentPossition = 0;
    private int slidePossition0 = 0;
    private int slidePossition1 = 1010;
    private int slidePossition2 = 1700;
    private int slidePossition3 = 2550;
    private boolean StartThread = true;






    @Override
    public void init() {
        DriverBot.init(hardwareMap);
        DriverBot.bucket.setPosition(0.002);
        DriverBot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }




    public void loop() {

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        DriverBot.RF.setPower(slow*(-pivot + (vertical - horizontal)));
        DriverBot.RB.setPower(slow*(-pivot + (vertical + horizontal)));
        DriverBot.LF.setPower(slow*(pivot + (vertical + horizontal)));
        DriverBot.LB.setPower(slow*(pivot + (vertical - horizontal)));

        if(gamepad1.dpad_down && slow == 1){
            slow = 0.5;
        }else if(gamepad1.dpad_down && slow == 0.5){
            slow = 1; 
        }

        if(gamepad2.x || gamepad1.x){
            DriverBot.Toggleintake(1);
        }
        if(gamepad2.y || gamepad1.y){
            DriverBot.Toggleintake(-1);
        }

        if(levelIndicator != 0){
            if(gamepad2.a || gamepad1.a){
                try {
                    DriverBot.Dumpbucket();
                }catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }


        if(gamepad2.b || gamepad1.b){
            DriverBot.carousel.setPower(-1);

        }

        else {
            DriverBot.carousel.setPower(0);
        }

        if(gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
            try {
                Thread.sleep(50);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                levelIndicator = 0;
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
            }
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                levelIndicator = levelIndicator - 1;
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
            }
            if (levelIndicator == -1) {
                levelIndicator = 3;
                try {
                    Thread.sleep(50);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
            }
            if(levelIndicator == 0){
                currentPossition = slidePossition0;
            }else if(levelIndicator == 1){
                currentPossition = slidePossition1;
            }else if(levelIndicator == 2){
                currentPossition = slidePossition2;
            }else if(levelIndicator == 3){
                currentPossition = slidePossition3;
            }
            DriverBot.lift.setTargetPosition(currentPossition);
            DriverBot.lift.setPower(1);
            DriverBot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(!DriverBot.lift.isBusy()){
            DriverBot.lift.setPower(0);
        }
        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

    }
}


