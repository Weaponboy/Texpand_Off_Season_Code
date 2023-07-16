package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collection_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collection_slow_power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.CurrentDraw;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.CurrentDrawSpike;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Current_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed_Reverse;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Horizontal;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check_Blue;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.OldCurrent;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.PivotPID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Current_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_FF;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Pivot_Target;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.RRXdist;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.RRYdist;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Slides_Reverse_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.SlowPoint;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Start_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Time_Difference;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Top_Pivot_PID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Vertical;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Xdist;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.XdistForStop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Ydist;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.YdistForStop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.abort;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.conefound;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.counterfornest;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.drivePID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_d;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_f;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_i;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.pivot_p;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.rotdist;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.rotdistForStop;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.slides_Power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.strafePID;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ticks_in_degrees;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Collect;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Out_Way;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Base_Pivot_Transfer_Pos;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_1;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_2;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_3;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_4;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.De_Pos_5;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Auto;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.High_Pole_Cycle;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Stop_Point;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Closed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Gripper_Open_Wide;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Nest_Position;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Pivot_Waiting_For_Cone;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints.Top_Turn_Middle;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry.X;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry.Y;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry.Odometry.heading;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.driveP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.rotationP;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeD;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeF;
import static org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.OdometryInit.MovePIDTuning.strafeP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous
public class Rebuild_Auto extends LinearOpMode {

    Odometry odometry = new Odometry();

    Drivetrain drive = new Drivetrain();

    Collection_Slides collectionSlides = new Collection_Slides();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Top_Gripper top = new Top_Gripper();

    Base_Gripper bottom = new Base_Gripper();

    ElapsedTime elapsedTime = new ElapsedTime();

    Sensors sensors = new Sensors();

    @Override
    public void runOpMode() throws InterruptedException {

        Init();

        drivePID = new PIDFController(driveP, 0, driveD, driveF);

        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);

        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        waitForStart();

        elapsedTime.reset();

        Odo_Drive(-110, -3, 36, true);

        DropPreLoad();

        AutoCycle();

        Odo_Drive(-110, 0, 0, false);

        while (opModeIsActive()){

            Top_Pivot_Position();

            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.addData("heading", Math.toDegrees(heading));
            telemetry.update();

        }
    }

    public void Init(){

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        sensors.init(hardwareMap, true);

        deliverySlides.init(hardwareMap);

        collectionSlides.init(hardwareMap);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        top.pivot_controller = new PIDController(pivot_p, pivot_i, pivot_d);

        bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

        top.Top_Gripper.setPosition(Top_Gripper_Closed);

        top.Top_Turn_Table.setPosition(Top_Turn_Middle);

    }

    public void Odo_Drive(double targetX, double targetY, double targetRot, boolean Preload) {

        do {

            Top_Pivot_Position();

            odometry.odometry();

            //GET CURRENT X
            double CurrentXPos = X;

            //GET CURRENT Y
            double CurrentYPos = Y;

            if (Preload && CurrentXPos > 75){
                ExtendHighPreloaded();
            }

            //GET START HEADING WITH ODOMETRY
            double Heading = Math.toDegrees(heading);

            //PID FOR DRIVING IN THE Y DIRECTION
            drivePID.setPIDF(driveP, 0, driveD, driveF);

            //PID FOR DRIVING IN THE X DIRECTION
            strafePID.setPIDF(strafeP, 0, strafeD, strafeF);

            //PID FOR TURNING
            PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

            //SET DISTANCE TO TRAVEL ERROR
            Xdist = (-targetX - CurrentXPos);
            Ydist = (targetY - CurrentYPos);

            XdistForStop = (-targetX - CurrentXPos);
            YdistForStop = (targetY - CurrentYPos);

            //CONVERT HEADING FOR TRIG CALCS
            if (Heading <= 0) {
                ConvertedHeading = (360 + Heading);
            } else {
                ConvertedHeading = (0 + Heading);
            }

            rotdist = (targetRot - Heading) * 1.55;

            rotdistForStop = (targetRot - Heading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 180) {
                rotdist = (rotdist - 360);
            }

            if (rotdistForStop < -180) {
                rotdistForStop = (360 + rotdistForStop);
            } else if (rotdistForStop > 180) {
                rotdistForStop = (rotdistForStop - 360);
            }

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            RRXdist = Xdist * Math.cos(Math.toRadians(360 - ConvertedHeading)) - Ydist * Math.sin(Math.toRadians(360 - ConvertedHeading));
            RRYdist = Xdist * Math.sin(Math.toRadians(360 - ConvertedHeading)) + Ydist * Math.cos(Math.toRadians(360 - ConvertedHeading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = drivePID.calculate(RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            //SET MOTOR POWER USING THE PID OUTPUT
            drive.RF.setPower(-Pivot + (Vertical + Horizontal));
            drive.RB.setPower((-Pivot * 1.4) + (Vertical - (Horizontal * 1.3)));
            drive.LF.setPower(Pivot + (Vertical - Horizontal));
            drive.LB.setPower((Pivot * 1.4) + (Vertical + (Horizontal * 1.3)));


        }while ((Math.abs(XdistForStop) > 0.8 ) || (Math.abs(YdistForStop) > 0.8 ) || (Math.abs(rotdistForStop) > 0.8));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void AutoCycle() {

        Odo_Drive(-129, -21, 85.5, false);

        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

        bottom.Base_Gripper.setPosition(Base_Gripper_Open);

        //cone 1
        CollectCone(De_Pos_1, -0);

        top.Top_Gripper.setPosition(0);

        Time = elapsedTime.milliseconds() > 27000;

        if (!abort && !Time) {

            top.Top_Gripper.setPosition(0);

            DropCycleCone();
        }

        Time = elapsedTime.milliseconds() > 27000;

        if (abort || Time) {

            Pivot_Target = Top_Pivot_Nest_Position;

            bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

        }else {

            Odo_Drive(-129, -21, 85.5, false);

            bottom.Base_Gripper.setPosition(Base_Gripper_Open);

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            //cone 2
            CollectCone(De_Pos_2, 0);

            top.Top_Gripper.setPosition(Top_Gripper_Closed);

            Time = elapsedTime.milliseconds() > 27000;

            if (!abort && !Time) {

                top.Top_Gripper.setPosition(0);

                DropCycleCone();
            }

            Time = elapsedTime.milliseconds() > 27000;

            if (abort || Time) {

                Pivot_Target = Top_Pivot_Nest_Position;

                bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

            }else {

                Odo_Drive(-129, -21, 85.5, false);

                bottom.Base_Gripper.setPosition(Base_Gripper_Open);

                bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

                //cone 3
                CollectCone(De_Pos_3, 0);

                top.Top_Gripper.setPosition(Top_Gripper_Closed);

                Time = elapsedTime.milliseconds() > 27000;

                if (!abort && !Time) {

                    top.Top_Gripper.setPosition(0);

                    DropCycleCone();
                }

                Time = elapsedTime.milliseconds() > 27000;

                if (abort || Time) {

                    Pivot_Target = Top_Pivot_Nest_Position;

                    bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                }else {

                    Odo_Drive(-129, -21, 85.5, false);

                    bottom.Base_Gripper.setPosition(Base_Gripper_Open);

                    bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

                    //cone 4
                    CollectCone(De_Pos_4, 0);

                    top.Top_Gripper.setPosition(Top_Gripper_Closed);

                    Time = elapsedTime.milliseconds() > 27000;

                    if (!abort && !Time) {

                        top.Top_Gripper.setPosition(0);

                        DropCycleCone();
                    }

                    Time = elapsedTime.milliseconds() > 27000;

                    if (abort || Time) {

                        Pivot_Target = Top_Pivot_Nest_Position;

                        bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                    }else {

                        Odo_Drive(-129, -21, 85.5, false);

                        bottom.Base_Gripper.setPosition(Base_Gripper_Open);

                        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

                        //cone 5
                        CollectCone(De_Pos_5, 0);

                        top.Top_Gripper.setPosition(Top_Gripper_Closed);

                        Time = elapsedTime.milliseconds() > 27000;

                        if (!abort && !Time) {

                            top.Top_Gripper.setPosition(0);

                            DropCycleCone();

                            bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);
                        }

                    }

                }

            }

        }
    }

    public void Destack_5 () {

        bottom.Base_Pivot.setPosition(0.2);

        Odo_Drive(132, -8, 220,false);

        bottom.Base_Gripper.setPosition(0.4);

        bottom.Base_Pivot.setPosition(0.1);

        //Collect Cone Position
        Odo_Drive(125, -28, 270,false);

        //cone 1
        CollectCone(De_Pos_1, 0);

        top.Top_Gripper.setPosition(0);

        Time = elapsedTime.milliseconds() > 27000;


        if (!abort && ! Time){
            top.Top_Gripper.setPosition(0);

            ExtendHigh();

            //Drop Off Position
            Odo_Drive(125, -15, 232,false);

            DropPreLoad();
        }

        Time = elapsedTime.milliseconds() > 27000;

        if (abort || Time){

            Pivot_Target = Top_Pivot_Nest_Position;

            bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

        }else {

            bottom.Base_Pivot.setPosition(0.2);

            //Collect Cone Position
            Odo_Drive(125, -28, 270,false);

            bottom.Base_Gripper.setPosition(Base_Gripper_Open);

            bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

            //cone 2
            CollectCone(0.2, 0);

            top.Top_Gripper.setPosition(0);

            Time = elapsedTime.milliseconds() > 27000;

            //Drop Off Position
            if (!abort && !Time){

                top.Top_Gripper.setPosition(0);

                ExtendHigh();

                //Drop Off Position
                Odo_Drive(125, -15, 232,false);

                DropPreLoad();
            }

            Time = elapsedTime.milliseconds() > 27000;

            if (abort || Time){

                Pivot_Target = Top_Pivot_Nest_Position;

                bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

            } else {

                bottom.Base_Pivot.setPosition(0.2);

                //Collect Cone Position
                Odo_Drive(125, -28, 270,false);

                bottom.Base_Gripper.setPosition(0.4);

                bottom.Base_Pivot.setPosition(0.05);

                //cone 3
                CollectCone(De_Pos_3, 0);

                top.Top_Gripper.setPosition(0);

                Time = elapsedTime.milliseconds() > 27000;

                if (!abort && !Time){

                    top.Top_Gripper.setPosition(0);

                    ExtendHigh();

                    //Drop Off Position
                    Odo_Drive(125, -15, 232,false);

                    DropPreLoad();

                }

                Time = elapsedTime.milliseconds() > 27000;

                if (abort || Time){

                    Pivot_Target = Top_Pivot_Nest_Position;

                    bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                }else{

                    bottom.Base_Pivot.setPosition(0.2);

                    //Collect Cone Position
                    Odo_Drive(125, -28, 270,false);

                    bottom.Base_Gripper.setPosition(0.4);

                    bottom.Base_Pivot.setPosition(0.05);

                    //cone 4
                    CollectCone(De_Pos_4, 0);

                    top.Top_Gripper.setPosition(0);

                    Time = elapsedTime.milliseconds() > 27000;

                    if (!abort && !Time){

                        top.Top_Gripper.setPosition(0);

                        ExtendHigh();

                        //Drop Off Position
                        Odo_Drive(125, -15, 232,false);

                        DropPreLoad();
                    }

                    Time = elapsedTime.milliseconds() > 27000;

                    if (abort || Time) {

                        Pivot_Target = Top_Pivot_Nest_Position;

                        bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                    }else{

                        bottom.Base_Pivot.setPosition(0.2);

                        //Collect Cone Position
                        Odo_Drive(125, -28, 270,false);

                        bottom.Base_Gripper.setPosition(0.4);

                        bottom.Base_Pivot.setPosition(0.05);

                        //cone 5
                        CollectCone(De_Pos_5, 0);

                        top.Top_Gripper.setPosition(0);

                        Time = elapsedTime.milliseconds() > 27000;

                        if (!abort && !Time){

                            top.Top_Gripper.setPosition(0);

                            ExtendHigh();

                            //Drop Off Position
                            Odo_Drive(125, -15, 232,false);

                            DropPreLoad();
                        }

                    }
                }
            }

        }

    }

    public void CollectCone(double De_pos, double TopConeDistance){

        Nest_Check = sensors.Nest_Check.blue() > Nest_Check_Blue;

        if (!Nest_Check){

            bottom.Base_Gripper.setPosition(0.4);

            bottom.Destacker_Left.setPosition(De_pos);
            bottom.Destacker_Right.setPosition(De_pos);

            if(bottom.Destacker_Left.getPosition() == De_Pos_1){
                bottom.Base_Pivot.setPosition(0.05);
            }else{
                bottom.Base_Pivot.setPosition(0.05);
            }

            Sleep(200);

            Pivot_Target = 190;

            collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            collectionSlides.Extend.setPower(Collection_Slides_Max_Speed);

            conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < 65 + TopConeDistance;

            CurrentDrawSpike = false;

            //extend till we find a cone or get to the slides limit
            while (!conefound && collectionSlides.Extend.getCurrentPosition() > (Stop_Point - 40) && !CurrentDrawSpike) {

//                OldCurrent = CurrentDraw;
//
//                CurrentDraw = collectionSlides.Extend.getCurrent(CurrentUnit.AMPS);
//
//                CurrentDrawSpike = (CurrentDraw - OldCurrent) > 1.2 && collectionSlides.Extend.getCurrentPosition() < -100 && !conefound;

                Slide_Position();

                Top_Pivot_Position();

                conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < 65 + TopConeDistance;

                SlowPoint = collectionSlides.Extend.getCurrentPosition() < -350;

                if (SlowPoint){
                    collectionSlides.Extend.setPower(Collection_slow_power);
                }else {
                    collectionSlides.Extend.setPower(Collection_Slides_Max_Speed);
                }

            }

            collectionSlides.Extend.setPower(0);

            conefound = sensors.Collect_Cone.getDistance(DistanceUnit.MM) < 65 + TopConeDistance;

            if (conefound || collectionSlides.Extend.getCurrentPosition() <= ((Stop_Point - 40) + 20)){

                //close gripper
                bottom.Base_Gripper.setPosition(Base_Gripper_Closed);

                Slide_Position();

                Sleep(200);

                bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                Sleep(225);

                collectionSlides.Extend.setTargetPosition(0);

                collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (collectionSlides.Extend.isBusy()) {

                    Slide_Position();

                    collectionSlides.Extend.setPower(0.9);

                    bottom.Base_Pivot.setPosition(Base_Pivot_Transfer_Pos);

                    if(collectionSlides.Extend.getCurrentPosition() > -200){
                        bottom.Destacker_Left.setPosition(De_Pos_5);
                        bottom.Destacker_Right.setPosition(De_Pos_5);

                    }
                    if(collectionSlides.Extend.getCurrentPosition() > -50){
                        bottom.Base_Gripper.setPosition(0.4);
                    }
                    if(collectionSlides.Extend.getCurrentPosition() > -70){

                        //open top gripper
                        top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

                        Pivot_Target = 0;
                    }
                }

                collectionSlides.Extend.setPower(0);

                //open base gripper
                bottom.Base_Gripper.setPosition(Base_Gripper_Open);

                Nest_Check = sensors.Nest_Check.blue() > Nest_Check_Blue;

                bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                Nest_Check = sensors.Nest_Check.blue() > 1500;

                if (!Nest_Check){
                    Sleep(200);
                }

                counterfornest = 0;

                while(!Nest_Check && elapsedTime.milliseconds() < 26000){

                    counterfornest++;

                    Sleep(100);

                    if (counterfornest == 5){

                        bottom.Base_Pivot.setPosition(0.55);

                        Sleep(400);

                        bottom.Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        Sleep(400);
                    }

                    Nest_Check = sensors.Nest_Check.blue() > Nest_Check_Blue;

                }


                if (Nest_Check) {

                    //close top gripper
                    top.Top_Gripper.setPosition(Top_Gripper_Closed);

                    Sleep(400);

                }else{
                    abort = true;
                }

            }else{

                bottom.Base_Pivot.setPosition(0.82);

                collectionSlides.Extend.setTargetPosition(0);

                collectionSlides.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (collectionSlides.Extend.isBusy()) {

                    Slide_Position();

                    collectionSlides.Extend.setPower(Slides_Reverse_Max_Speed);

                }
                collectionSlides.Extend.setPower(0);

                bottom.Destacker_Left.setPosition(De_Pos_5);
                bottom.Destacker_Right.setPosition(De_Pos_5);

                abort = true;
            }

        }else {
            abort = true;
        }

    }

    public void ExtendHigh (){

        top.Top_Gripper.setPosition(0);

        Pivot_Target = 885;

        Top_Pivot_Position();

        deliverySlides.DeliverySlides(High_Pole_Auto, Delivery_Slides_Max_Speed);

        top.Top_Turn_Table.setPosition(0.35);

    }

    public void ExtendHighPreloaded(){

        top.Top_Gripper.setPosition(0);

        Pivot_Target = 650;

        Top_Pivot_Position();

        deliverySlides.DeliverySlides(High_Pole_Cycle, Delivery_Slides_Max_Speed);

    }

    public void DropPreLoad(){

        while (deliverySlides.Right_Slide.getCurrentPosition() < 600){
            Sleep(50);
        }

        Sleep(200);

        Pivot_Target = 950;

        Top_Pivot_Position();

        while (top.Top_Pivot.getCurrentPosition() < 900){
            Sleep(50);
        }

        Sleep(200);

        top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

        //TO DO: Insert WHILE loop
        if (top.Top_Gripper.getPosition() == Top_Gripper_Open_Wide){

            Sleep(50);

            Pivot_Target = 200;

            deliverySlides.DeliverySlides(0, Delivery_Slides_Max_Speed_Reverse);

            slides_Power = true;
        }
    }

    public void DropCycleCone(){

//        collectionSlides.Collect_RunToPosition(-200, Collection_Slides_Max_Speed);

        bottom.Base_Pivot.setPosition(Base_Pivot_Collect);

        deliverySlides.DeliverySlides(High_Pole_Auto, Delivery_Slides_Max_Speed);

        while (deliverySlides.Right_Slide.getCurrentPosition() < 910){

            Pivot_Target = 850;

            Top_Pivot_Position();

            top.Top_Turn_Table.setPosition(0.33);

        }

        Sleep(400);

        Pivot_Target = 1000;

        Top_Pivot_Position();

        Sleep(300);

        top.Top_Gripper.setPosition(Top_Gripper_Open);

        Sleep(200);

        deliverySlides.DeliverySlides(0, Delivery_Slides_Max_Speed_Reverse);

        top.Top_Turn_Table.setPosition(Top_Turn_Middle);

        top.Top_Gripper.setPosition(Top_Gripper_Open_Wide);

        Pivot_Target = Top_Pivot_Waiting_For_Cone;

        slides_Power = true;
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

            Slide_Position();

            Current_Time = elapsedTime.milliseconds();

            Time_Difference = Current_Time - Start_Time;

        }

    }

}
