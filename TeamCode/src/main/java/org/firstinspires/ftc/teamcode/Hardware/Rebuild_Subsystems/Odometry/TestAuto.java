package org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Odometry;

import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collection_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Collection_slow_power;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.ConvertedHeading;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Current_Time;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Delivery_Slides_Max_Speed_Reverse;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Horizontal;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check;
import static org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants.Nest_Check_Blue;
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

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Base_Gripper;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Collection_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Sensors;
import org.firstinspires.ftc.teamcode.Hardware.Rebuild_Subsystems.Top_Gripper;

@Autonomous
public class TestAuto extends LinearOpMode {

    Odometry odometry = new Odometry();

    Drivetrain drive = new Drivetrain();

    public static PIDFController drivePID;
    public static PIDFController strafePID;
    public static PIDFController PivotPID;

    @Override
    public void runOpMode() throws InterruptedException{

        drivePID = new PIDFController(driveP, 0, driveD, driveF);
        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        waitForStart();


        while(opModeIsActive()){

            double targetX = 10;

            double targetY = 40;

            double targetRot = 0;

            odometry.odometry();

            //GET CURRENT X
            double CurrentXPos = X;

            //GET CURRENT Y
            double CurrentYPos = Y;

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

            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.addData("heading", Math.toDegrees(heading));
            telemetry.update();

        }
    }

}
