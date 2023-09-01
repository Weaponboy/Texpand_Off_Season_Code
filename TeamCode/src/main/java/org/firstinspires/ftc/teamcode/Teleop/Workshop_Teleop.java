package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Auto_Collect_Cone_Distance;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Current_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed_Reverse;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Full_Cycle_Toggle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check_Blue;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Current_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_FF;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Target;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Start_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.TargetPixels;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Time_Difference;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Top_Pivot_PID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Trigger_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.conefound_Noslides;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.deltaServoPosition;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.heading;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_f;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.servoPosition;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.throttle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Flip;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Out_Way;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Transfer_Pos;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_5;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Driver;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open_Wide;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Almost_Drop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Deliver;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Nest_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Waiting_For_Cone;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Turn_Middle;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry.X;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry.Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Base_Gripper;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Collection_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Top_Gripper;

import java.util.List;

@TeleOp
public class Workshop_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odo = new Odometry();

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

        throttle = 0.3;

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x*1.5;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower(throttle*(-pivot + (vertical - horizontal)));
        drive.RB.setPower((throttle*1.15)*(-pivot + (vertical + horizontal)));
        drive.LF.setPower(throttle*(pivot + (vertical + horizontal)));
        drive.LB.setPower((throttle*1.15)*(pivot + (vertical - horizontal)));

        /**Collection slides*/

        conefound_Noslides = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < Auto_Collect_Cone_Distance && !Trigger_Collect && !Full_Cycle_Toggle;

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

        if (top.Top_Pivot.getCurrentPosition() < 30 && sensors.Nest_Check.blue() > Nest_Check_Blue){
            bottom.Destacker_Position(De_Pos_5);
            top.Top_Gripper.setPosition(Top_Gripper_Closed);
        }

        /**Top pivot position*/

        if (gamepad1.x) {
            Pivot_Target = Top_Pivot_Almost_Drop;
        }

        if (gamepad1.start) {
            Pivot_Target = Top_Pivot_Waiting_For_Cone;
        }

        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Pivot_Target = Top_Pivot_Deliver;
        }

        /** Delivery slides*/

        if(gamepad2.right_trigger > 0 || gamepad1.right_trigger > 0){

            deliverySlides.DeliverySlides(0, Delivery_Slides_Max_Speed_Reverse);

            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

            if (bottom.Destacker_Left.getPosition() > 0.9){
                Pivot_Target = Top_Pivot_Nest_Position;
            }else {
                Pivot_Target = Top_Pivot_Waiting_For_Cone;
            }


            slides_Power = true;

        }

        /** Manual controls*/

        if(currentGamepad1.y && !previousGamepad1.y && top.Top_Gripper.getPosition() == 0){
            top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);
        }else if(currentGamepad1.y && !previousGamepad1.y && top.Top_Gripper.getPosition() > 0){
            top.Top_Gripper.setPosition(Top_Gripper_Closed);
        }

        if(currentGamepad1.a && !previousGamepad1.a && bottom.Base_Pivot.getPosition() < 0.45){
            bottom.Base_Pivot.setPosition(Base_Pivot_Flip);
        }else if(currentGamepad1.a && !previousGamepad1.a && bottom.Base_Pivot.getPosition() > 0){
            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);
        }

        if(currentGamepad1.b && !previousGamepad1.b && bottom.Base_Gripper.getPosition() == Base_Gripper_Closed) {
            bottom.Base_Gripper.setPosition(Base_Gripper_Open);
        }else if(currentGamepad1.b && !previousGamepad1.b && bottom.Base_Gripper.getPosition() > Base_Gripper_Closed && bottom.Base_Pivot.getPosition() < 0.4){
            bottom.Base_Gripper.setPosition(Base_Gripper_Closed);
        }

        Slide_Position();

        Top_Pivot_Position();


    }

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        top.Top_Turn_Table.setPosition(Top_Turn_Middle);

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
