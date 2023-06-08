package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Auto_Collect_Cone_Distance;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collect_Cone_Distance;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Current_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed_Reverse;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Full_Cycle_Toggle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check_Blue;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Current_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_FF;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Target;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Slides_Safety_Stop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.SlowPoint;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Start_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Time_Difference;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Top_Pivot_PID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Trigger_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.collecting_cone;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.conefound;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.conefound_Noslides;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.heading;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_f;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_slow_point;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Flip;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Out_Way;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Transfer_Pos;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_1;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_3;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_4;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_5;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Drop_Cone_Cycle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Auto;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Cycle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Cycle_while;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Driver;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Stop_Point;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open_Wide;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Almost_Drop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Deliver;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Base_Gripper;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Collection_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Top_Gripper;

import java.util.List;

@TeleOp
public class Rebuild_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

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

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void loop() {

        /**Drive code*/
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        throttle = 0.6;

        throttle = (gamepad1.left_trigger * 0.4) + throttle;

        if (gamepad1.right_trigger > 0) {
            throttle = 0.3;
        }

        drive.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower((throttle *1.15)*(-pivot + (vertical - horizontal)));
        drive.RB.setPower(throttle *(-pivot + (vertical + horizontal)));
        drive.LF.setPower((throttle *1.15)*(pivot + (vertical + horizontal)));
        drive.LB.setPower(throttle *(pivot + (vertical - horizontal)));

        /**Collection slides*/

        if(gamepad2.left_trigger > 0){

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectionSlides.Extend.setPower(-0.6);

            Trigger_Collect = true;
        }

        if (gamepad2.left_stick_button){
            Full_Cycle_Toggle = true;
        }else if(gamepad2.right_stick_button){
            Full_Cycle_Toggle = false;
        }

        conefound_Noslides = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < Auto_Collect_Cone_Distance && !Trigger_Collect && !Full_Cycle_Toggle;

        conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < Collect_Cone_Distance;

        Slides_Safety_Stop = collectionSlides.Extend.getCurrentPosition() < Stop_Point;

        SlowPoint = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < slides_slow_point;

        if (SlowPoint && collectionSlides.Extend.getVelocity() < 0 && collectionSlides.Extend.getCurrentPosition() < -50){
            collectionSlides.Extend.setPower(-0.2);
        }

        if (conefound_Noslides && bottom.Base_Pivot.getPosition() < 0.4){

            if (drive.LB.getPower() > 0){
                drive.LB.setPower(-0.1);
                drive.LF.setPower(-0.1);
                drive.RF.setPower(-0.1);
                drive.RB.setPower(-0.1);
            }

            bottom.Base_Gripper.setPosition(Base_Gripper_Closed);

            Sleep(200);

            bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

            Sleep(500);

            bottom.Base_Gripper.setPosition(Base_Gripper_Open);

            Sleep(50);

            bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

            bottom.Destacker_Position(De_Pos_5);

        }

        if (Slides_Safety_Stop && !conefound){
            collectionSlides.Extend.setPower(0);

            collectionSlides.Extend.setTargetPosition(0);

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            collectionSlides.Extend.setPower(0.6);
        }

        if (conefound && bottom.Base_Pivot.getPosition() < 0.2 && collectionSlides.Extend.getVelocity() < 0){

            collectionSlides.Extend.setPower(0);

            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }

            Slide_Position();

            bottom.Base_Gripper.setPosition(Base_Gripper_Closed);

            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Slide_Position();

            Pivot_Target = Top_Pivot_Waiting_For_Cone;

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

        if(collectionSlides.Extend.getCurrentPosition() > -200 && collectionSlides.Extend.getVelocity() > 0){
            bottom.Destacker_Left.setPosition(De_Pos_5);
            bottom.Destacker_Right.setPosition(De_Pos_5);
        }

        if(collectionSlides.Extend.getCurrentPosition() > -20 && collecting_cone) {

            Pivot_Target = Top_Pivot_Nest_Position;

            Top_Pivot_Position();

            bottom.Base_Gripper.setPosition(Base_Gripper_Open);

            bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

            collecting_cone = false;

            Trigger_Collect = false;

        }

        if (top.Top_Pivot.getCurrentPosition() < 30 && sensors.Nest_Check.blue() > Nest_Check_Blue){
            bottom.Destacker_Position(De_Pos_5);
            top.Top_Gripper.setPosition(Top_Gripper_Closed);
        }

        /**Full cycle*/

        if(gamepad2.back){

            top.Top_Turn_Table.setPosition(Top_Turn_Middle);

            bottom.Base_Gripper.setPosition(Base_Gripper_Closed);

            Sleep(200);

            bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

            Sleep(400);

            bottom.Base_Gripper.setPosition(Base_Gripper_Open);

            Sleep(50);

            bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

            Sleep(600);
            
            top.Top_Gripper.setPosition(Top_Gripper_Closed);

            Sleep(600);

            deliverySlides.DeliverySlides(High_Pole_Cycle, Delivery_Slides_Max_Speed);
            
            while (deliverySlides.Right_Slide.getCurrentPosition() < High_Pole_Cycle_while){

                deliverySlides.Right_Slide.setPower(Delivery_Slides_Max_Speed);
                deliverySlides.Left_Slide.setPower(Delivery_Slides_Max_Speed);

                Pivot_Target = Drop_Cone_Cycle;

                Top_Pivot_Position();
            }

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            Top_Pivot_Position();

            Sleep(200);

            top.Top_Gripper.setPosition(Top_Gripper_Open);

            Sleep(200);

            deliverySlides.DeliverySlides(0, Delivery_Slides_Max_Speed_Reverse);

            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

            Pivot_Target = Top_Pivot_Nest_Position;

            slides_Power = true;

        }

        /**Top pivot position*/

        //Top Pivot nest position
        if (gamepad2.x) {
            top.Top_Turn_Table.setPosition(Top_Turn_Middle);

            Pivot_Target = 0;
        }

        //Top pivot ready to drop
        if (gamepad1.x) {
            Pivot_Target = Top_Pivot_Almost_Drop;
        }

        if (gamepad1.start) {
            Pivot_Target = Top_Pivot_Waiting_For_Cone;
        }

        //Top Pivot Drop cone position
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Pivot_Target = Top_Pivot_Deliver;
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

        //High pole set point
        if(gamepad2.left_bumper){
            deliverySlides.DeliverySlides(High_Pole_Driver, Delivery_Slides_Max_Speed);
        }

        //Auto test button
        if(gamepad2.dpad_left){

            deliverySlides.Right_Slide.setTargetPosition(High_Pole_Auto);
            deliverySlides.Left_Slide.setTargetPosition(High_Pole_Auto);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            deliverySlides.Right_Slide.setPower(1);
            deliverySlides.Left_Slide.setPower(1);

            while (deliverySlides.Right_Slide.getCurrentPosition() < 870){

                Pivot_Target = 885;

                Top_Pivot_Position();

                top.Top_Turn_Table.setPosition(0.35);

            }

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Pivot_Target = 1000;

            Top_Pivot_Position();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Top_Pivot_Position();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Top_Pivot_Position();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            top.Top_Gripper.setPosition(Top_Gripper_Open);

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            deliverySlides.Right_Slide.setTargetPosition(0);
            deliverySlides.Left_Slide.setTargetPosition(0);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            deliverySlides.Right_Slide.setPower(-0.9);
            deliverySlides.Left_Slide.setPower(-0.9);

            top.Top_Turn_Table.setPosition(Top_Turn_Middle);

            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

            Pivot_Target = Top_Pivot_Waiting_For_Cone;

            slides_Power = true;

        }

        //Return to zero
        if(gamepad2.right_trigger > 0){

            deliverySlides.DeliverySlides(0, Delivery_Slides_Max_Speed_Reverse);

            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

            Pivot_Target = Top_Pivot_Waiting_For_Cone;

            slides_Power = true;

        }

        /** Manual controls*/

        if (gamepad1.dpad_right && bottom.Base_Pivot.getPosition() <= 0.3){
            bottom.Base_Pivot.setPosition(0.38);
        }else if (gamepad1.dpad_right && bottom.Base_Pivot.getPosition() == 0.38){
            bottom.Base_Pivot.setPosition(0.3);
        }

        if(currentGamepad2.y && !previousGamepad2.y && top.Top_Gripper.getPosition() == 0){
            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);
        }else if(currentGamepad2.y && !previousGamepad2.y && top.Top_Gripper.getPosition() > 0){
            top.Top_Gripper.setPosition(Top_Gripper_Closed);
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

        if (gamepad1.dpad_up && top.Top_Pivot.getCurrentPosition() > 180) {
            bottom.Destacker_Position(De_Pos_1);
            bottom.Base_Pivot.setPosition(0.3);
        }
        if (gamepad1.right_bumper && top.Top_Pivot.getCurrentPosition() > 180) {
            bottom.Destacker_Position(De_Pos_1);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.y && top.Top_Pivot.getCurrentPosition() > 180) {
            bottom.Destacker_Position(De_Pos_3);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.b && top.Top_Pivot.getCurrentPosition() > 180) {
            bottom.Destacker_Position(De_Pos_4);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }
        if (gamepad1.a && top.Top_Pivot.getCurrentPosition() > 180) {
            bottom.Destacker_Position(De_Pos_5);
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }

        Slide_Position();

        Top_Pivot_Position();

        odo.odometry.updatePose();

        heading = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("Top Pivot", bottom.Base_Pivot.getPosition());
        telemetry.addData("Full cycle toggle", Full_Cycle_Toggle);
        telemetry.addData("Distance sensor", sensors.Collect_Cone.getDistance(DistanceUnit.MM));
        telemetry.addData("Top_Pivot Target", Pivot_Target);
        telemetry.addData("Top_Pivot", top.Top_Pivot.getCurrentPosition());
        telemetry.addData("Extend ticks", collectionSlides.Extend.getCurrentPosition());
        telemetry.addData("Extend velocity", collectionSlides.Extend.getVelocity());
        telemetry.addData("Extend velocity", sensors.pole.getTargetHighrectX());
        telemetry.addData("Pivot current draw", top.Top_Pivot.getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addData("heading", heading.firstAngle);
        telemetry.addData("X", odo.getXpos());
        telemetry.addData("Y", odo.getYpos());

        telemetry.addData("Left pod", odo.leftOdometer.getPosition());
        telemetry.addData("Right pod", odo.rightOdometer.getPosition());
        telemetry.addData("center pod", odo.centerOdometer.getPosition());
        telemetry.addData("Extend", collectionSlides.Extend.getCurrentPosition());
        telemetry.addData("Right slide", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("Left slide", deliverySlides.Left_Slide.getCurrentPosition());
        telemetry.addData("Blue", sensors.Nest_Check.blue());
        telemetry.update();


    }

    @Override
    public void init() {

        odo.init(hardwareMap);

        drive.init(hardwareMap);

        sensors.init(hardwareMap, true);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collectionSlides.init(hardwareMap);

        top.pivot_controller = new PIDController(pivot_p, pivot_i, pivot_d);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Pivot_Target = Top_Pivot_Nest_Position;

        elapsedTime.reset();

    }

    public void Slide_Position(){

        if(deliverySlides.Right_Slide.getCurrentPosition() < 10 && !deliverySlides.Right_Slide.isBusy() && deliverySlides.Left_Slide.getCurrentPosition() < 10 && !deliverySlides.Left_Slide.isBusy()){
            deliverySlides.Right_Slide.setPower(0);
            deliverySlides.Left_Slide.setPower(0);

            deliverySlides.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deliverySlides.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slides_Power = false;

        }else if(slides_Power){

            deliverySlides.Right_Slide.setPower(Delivery_Slides_Max_Speed_Reverse);
            deliverySlides.Left_Slide.setPower(Delivery_Slides_Max_Speed_Reverse);

        }

    }

    public void Top_Pivot_Position(){

        top.pivot_controller.setPID(pivot_p, pivot_i, pivot_d);

        Pivot_Current_Position = top.Top_Pivot.getCurrentPosition();

        Top_Pivot_PID = top.pivot_controller.calculate(Pivot_Current_Position, Pivot_Target) * 0.8;

        Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;

        Pivot_Power = Top_Pivot_PID + Pivot_FF;

        top.Top_Pivot.setPower(Pivot_Power);

    }

    public void Sleep(double Sleep_Time){

        Start_Time = elapsedTime.milliseconds();

        Current_Time = elapsedTime.milliseconds();

        Time_Difference = Current_Time - Start_Time;

        while (Time_Difference < Sleep_Time){

            Top_Pivot_Position();

            Current_Time = elapsedTime.milliseconds();

            Time_Difference = Current_Time - Start_Time;

        }

    }

}
