package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collect_Cone_Distance;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Current_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_FF;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Target;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Slides_Reverse_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Slides_Safety_Stop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.SlowPoint;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Top_Pivot_PID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.brake;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.collecting_cone;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.conefound;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_f;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_slow_point;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slow_power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Flip;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Out_Way;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_1;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_2;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_3;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_4;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_5;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Drop_Cone_Cycle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Cycle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Cycle_while;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Driver;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Medium_Pole;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Stop_Point;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open_Wide;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Nest_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Waiting_For_Cone;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Turn_Left;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Turn_Middle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Turn_Right;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Base_Gripper;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Collection_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Drivetrain_2;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Top_Gripper;

import java.util.List;

@TeleOp
public class Rebuild_Teleop extends OpMode {

    Drivetrain_2 drive = new Drivetrain_2();

    OdometryInit odo = new OdometryInit();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection_Slides collectionSlides = new Collection_Slides();

    Sensors sensors = new Sensors();

    Top_Gripper top = new Top_Gripper();

    Base_Gripper bottom = new Base_Gripper();

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void loop() {

        Top_Pivot_Position();

        /**Drive code*/
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        brake = (-gamepad1.right_trigger * -0.5) + 0.5;

        throttle = (gamepad1.left_trigger * 0.6) + brake;

        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower((throttle *1.4)*(-pivot + (vertical - horizontal)));
        drive.RB.setPower(throttle *(-pivot + (vertical + horizontal)));
        drive.LF.setPower((throttle *1.4)*(pivot + (vertical + horizontal)));
        drive.LB.setPower(throttle *(pivot + (vertical - horizontal)));

        /**Collection slides*/

        if(gamepad2.left_trigger > 0){

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectionSlides.Extend.setPower(-0.6);

        }

        conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < 70;

        Slides_Safety_Stop = collectionSlides.Extend.getCurrentPosition() < Stop_Point;

        SlowPoint = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < 200;

//        if (SlowPoint && collectionSlides.Extend.getVelocity() < 0){
//            collectionSlides.Extend.setPower(-0.3);
//        }

        if (Slides_Safety_Stop && !conefound){
            collectionSlides.Extend.setPower(0);

            collectionSlides.Extend.setTargetPosition(0);

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            collectionSlides.Extend.setPower(0.6);
        }

        if (conefound && bottom.Base_Pivot.getPosition() < 0.1 && collectionSlides.Extend.getVelocity() < 0){

            collectionSlides.Extend.setPower(0);

            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            Slide_Position();

            bottom.Base_Gripper.setPosition(0);

            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Slide_Position();

            Pivot_Target = 200;

            Top_Pivot_Position();

            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

            bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

            if (bottom.Destacker_Left.getPosition() < 0.6){

                try {
                    Thread.sleep(400);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Pivot_Position();

            }

            Top_Pivot_Position();

            try {
                Thread.sleep(125);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Pivot_Position();

            collectionSlides.Extend.setTargetPosition(0);

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            collectionSlides.Extend.setPower(0.8);

            collecting_cone = true;

        }

        if(collectionSlides.Extend.getCurrentPosition() > -200 && collectionSlides.Extend.getPower() > 0){
            bottom.Destacker_Left.setPosition(De_Pos_5);
            bottom.Destacker_Right.setPosition(De_Pos_5);
        }

        if(bottom.Base_Pivot.getPosition() > 0.7 && collectionSlides.Extend.getVelocity() == 0 && collectionSlides.Extend.getCurrentPosition() > -10 && collecting_cone) {

            collectionSlides.Extend.setPower(0);

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Pivot_Target = 0;

            Top_Pivot_Position();

            bottom.Base_Gripper.setPosition(0.4);

            bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

            collecting_cone = false;

        }

        if (top.Top_Pivot.getCurrentPosition() < 5 && sensors.Nest_Check.blue() > 1500){

            top.Top_Gripper.setPosition(0);
        }

        /**Full cycle*/

        if (gamepad2.back){

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectionSlides.Extend.setPower(Slides_Max_Speed);

            conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < Collect_Cone_Distance;

            while(!conefound && collectionSlides.Extend.getCurrentPosition() > Stop_Point){

                Slide_Position();

                conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < Collect_Cone_Distance;

                SlowPoint = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < slides_slow_point;

                if (SlowPoint){
                    collectionSlides.Extend.setPower(slow_power);
                }else {
                    collectionSlides.Extend.setPower(Slides_Max_Speed);
                }

                Top_Pivot_Position();

            }

            collectionSlides.Extend.setPower(0);

            if(conefound) {

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                bottom.Base_Gripper.setPosition(Base_Gripper_Closed);

                try {
                    Thread.sleep(300);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Slide_Position();

                Pivot_Target = Top_Pivot_Waiting_For_Cone;

                Top_Pivot_Position();

                top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

                Top_Pivot_Position();

                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Pivot_Position();

                collectionSlides.Extend.setTargetPosition(0);

                collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                collectionSlides.Extend.setPower(Slides_Reverse_Max_Speed);

                while (collectionSlides.Extend.isBusy()) {

                    Slide_Position();

                    bottom.Base_Pivot.setPosition(Base_Pivot_Flip);

                    bottom.Base_Gripper.setPosition(0);

                    collectionSlides.Extend.setPower(Slides_Reverse_Max_Speed);

                    Top_Pivot_Position();

                    if(collectionSlides.Extend.getCurrentPosition() > -40){

                        //open top gripper
                        top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

                        //take top pivot to pick up the cone
                        Pivot_Target = Top_Pivot_Nest_Position;

                        Top_Pivot_Position();
                    }
                }

                Slide_Position();

                collectionSlides.Extend.setPower(0);

                bottom.Destacker_Position(De_Pos_5);

                    try {
                        Thread.sleep(100);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    bottom.Base_Gripper.setPosition(Base_Gripper_Open);

                    try {
                        Thread.sleep(150);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                    try {
                        Thread.sleep(50);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    Pivot_Target = Top_Pivot_Nest_Position;

                    Top_Pivot_Position();

                    try {
                        Thread.sleep(50);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    Top_Pivot_Position();

                    top.Top_Gripper.setPosition(Top_Gripper_Closed);

                    try {
                        Thread.sleep(100);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    Pivot_Target = Top_Pivot_Waiting_For_Cone;

                Top_Pivot_Position();

                    try {
                        Thread.sleep(50);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                Top_Pivot_Position();

                bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

                collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                deliverySlides.Right_Slide.setTargetPosition(High_Pole_Cycle);
                deliverySlides.Left_Slide.setTargetPosition(High_Pole_Cycle);

                deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                deliverySlides.Left_Slide.setPower(1);
                deliverySlides.Right_Slide.setPower(1);

                while (deliverySlides.Right_Slide.getCurrentPosition() < High_Pole_Cycle_while && deliverySlides.Left_Slide.getCurrentPosition() < High_Pole_Cycle_while) {

                    deliverySlides.Right_Slide.setPower(1);
                    deliverySlides.Left_Slide.setPower(1);

                    Slide_Position();

                }

                Pivot_Target = Drop_Cone_Cycle;

                Top_Pivot_Position();

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Top_Pivot_Position();

                deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

                deliverySlides.Right_Slide.setTargetPosition(0);
                deliverySlides.Left_Slide.setTargetPosition(0);

                deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Pivot_Target = Top_Pivot_Waiting_For_Cone;

                Top_Pivot_Position();

                deliverySlides.Right_Slide.setPower(-0.9);
                deliverySlides.Left_Slide.setPower(-0.9);

                slides_Power = true;

            }else{
                collectionSlides.Extend.setTargetPosition(0);

                collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                collectionSlides.Extend.setPower(Slides_Reverse_Max_Speed);

                Slide_Position();

                bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
            }



        }

        /**Top pivot position*/

        //Top Pivot nest position
        if (gamepad2.x) {
            Pivot_Target = 0;
        }

        //Top pivot ready to drop
        if (gamepad1.x) {
            Pivot_Target = 800;
        }

        //Top Pivot Drop cone position
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Pivot_Target = 1000;
        }

        /**Top turret positions*/
        if (gamepad2.left_stick_x > 0) {
            top.Top_Turn_Table.setPosition(Top_Turn_Right);
        }

        if (gamepad2.left_stick_x < 0) {
            top.Top_Turn_Table.setPosition(Top_Turn_Left);
        }

        if(gamepad2.left_stick_y < 0){
            top.Top_Turn_Table.setPosition(Top_Turn_Middle);
        }

        /** Delivery slides*/
        //High pole
        if(gamepad2.left_bumper){

            deliverySlides.Right_Slide.setTargetPosition(High_Pole_Driver);
            deliverySlides.Left_Slide.setTargetPosition(High_Pole_Driver);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            deliverySlides.Right_Slide.setPower(1);
            deliverySlides.Left_Slide.setPower(1);

        }

        //Medium pole
        if(gamepad2.right_bumper){

            deliverySlides.Right_Slide.setTargetPosition(Medium_Pole);
            deliverySlides.Left_Slide.setTargetPosition(Medium_Pole);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            deliverySlides.Right_Slide.setPower(1);
            deliverySlides.Left_Slide.setPower(1);

        }

        //Return to zero
        if(gamepad2.right_trigger > 0){

            deliverySlides.Right_Slide.setTargetPosition(0);
            deliverySlides.Left_Slide.setTargetPosition(0);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            deliverySlides.Right_Slide.setPower(-0.9);
            deliverySlides.Left_Slide.setPower(-0.9);

            slides_Power = true;

        }

        /** Manual controls*/

        if(currentGamepad2.y && !previousGamepad2.y && top.Top_Gripper.getPosition() == 0){
            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide); //lift up top griper if it is down
        }else if(currentGamepad2.y && !previousGamepad2.y && top.Top_Gripper.getPosition() > 0){
            top.Top_Gripper.setPosition(Top_Gripper_Closed);//lower top gripper if it is up
        }

        if(currentGamepad2.a && !previousGamepad2.a && bottom.Base_Pivot.getPosition() < 0.45){
            bottom.Base_Pivot.setPosition(Base_Pivot_Flip);
        }else if(currentGamepad2.a && !previousGamepad2.a && bottom.Base_Pivot.getPosition() > 0){
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }

        if(currentGamepad2.b && !previousGamepad2.b && bottom.Base_Gripper.getPosition() == Base_Gripper_Closed) {
            bottom.Base_Gripper.setPosition(Base_Gripper_Open);
        }else if(currentGamepad2.b && !previousGamepad2.b && bottom.Base_Gripper.getPosition() > Base_Gripper_Closed){
            bottom.Base_Gripper.setPosition(Base_Gripper_Closed);
        }

        /** Destacker Position */
        if (gamepad1.dpad_up) {
            bottom.Destacker_Left.setPosition(De_Pos_1);
            bottom.Destacker_Right.setPosition(De_Pos_1);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.right_bumper) {
            bottom.Destacker_Left.setPosition(De_Pos_2);
            bottom.Destacker_Right.setPosition(De_Pos_2);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.y) {
            bottom.Destacker_Left.setPosition(De_Pos_3);
            bottom.Destacker_Right.setPosition(De_Pos_3);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.b) {
            bottom.Destacker_Left.setPosition(De_Pos_4);
            bottom.Destacker_Right.setPosition(De_Pos_4);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.a) {
            bottom.Destacker_Left.setPosition(De_Pos_5);
            bottom.Destacker_Right.setPosition(De_Pos_5);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }

        Slide_Position();

        odo.odometry.updatePose();

        telemetry.addData("Top_Pivot Target", Pivot_Target);
        telemetry.addData("Top_Pivot", top.Top_Pivot.getCurrentPosition());
        telemetry.addData("Extend ticks", collectionSlides.Extend.getCurrentPosition());
        telemetry.addData("Extend velocity", collectionSlides.Extend.getVelocity());

        telemetry.addData("X", odo.getXpos());
        telemetry.addData("Y", odo.getYpos());

        telemetry.addData("Extend", collectionSlides.Extend.getCurrentPosition());
        telemetry.addData("Right slide", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("Left slide", deliverySlides.Left_Slide.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        sensors.init(hardwareMap, true);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collectionSlides.init(hardwareMap);

        odo.Odometryinit(this, hardwareMap);

        top.pivot_controller = new PIDController(pivot_p, pivot_i, pivot_d);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Pivot_Target = 0;

    }

    public void Slide_Position(){

        if(deliverySlides.Right_Slide.getCurrentPosition() < 10 && !deliverySlides.Right_Slide.isBusy() && deliverySlides.Left_Slide.getCurrentPosition() < 10 && !deliverySlides.Left_Slide.isBusy()){
            deliverySlides.Right_Slide.setPower(0);
            deliverySlides.Left_Slide.setPower(0);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slides_Power = false;

        }else if(slides_Power){

            deliverySlides.Right_Slide.setPower(-0.9);
            deliverySlides.Left_Slide.setPower(-0.9);

        }

    }

    public void Top_Pivot_Position(){

        top.pivot_controller.setPID(pivot_p, pivot_i, pivot_d);

        Pivot_Current_Position = top.Top_Pivot.getCurrentPosition();

        Top_Pivot_PID = top.pivot_controller.calculate(Pivot_Current_Position, Pivot_Target);

        Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;

        Pivot_Power = Top_Pivot_PID + Pivot_FF;

        top.Top_Pivot.setPower(Pivot_Power);

    }

}
