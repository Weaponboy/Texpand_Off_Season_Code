package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Servo_Test extends OpMode {

    public Servo Test_Servo = null;

    @Override
    public void init() {
        Test_Servo = hardwareMap.get(Servo.class,"Test_Servo");
        Test_Servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0){
            Test_Servo.setPosition(1);
            try {
                sleep(1000);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Test_Servo.setPosition(0.5);
        }

    }
}
