package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Constants;
import org.firstinspires.ftc.teamcode.ConstantsAndSetPoints.Setpoints;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;

@TeleOp
public class DoubleGripperRed extends OpMode {
    private double vertical;
    private double horizontal;
    private double pivot;

    Constants constants = new Constants();

    Setpoints setpoints = new Setpoints();

    Drivetrain drive = new Drivetrain();

    Slides slide = new Slides(telemetry);

    Bottom_Gripper_Assembly bottom = new Bottom_Gripper_Assembly();

    Top_gripper top = new Top_gripper();

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void loop() {
        this.runtime.reset();

        constants.slow1 = (gamepad1.left_trigger * 0.6) + 0.4;

        drive.WithOutEncoders();

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        drive.RF.setPower(constants.slow1*1.3*(-pivot + (vertical - horizontal)));
        drive.RB.setPower(constants.slow1*(-pivot + (vertical + horizontal)));
        drive.LF.setPower(constants.slow1*1.3*(pivot + (vertical + horizontal)));
        drive.LB.setPower(constants.slow1*(pivot + (vertical - horizontal)));

        //destacker toggle
        if (gamepad2.dpad_up) {
            ToggleDestackPosition();
        }

        //Toggle base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
            B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        //Sleep
        sleep(10);

        //Destacker position 1
        if (gamepad1.dpad_up) {
            bottom.Destacker_Left.setPosition(setpoints.De_Pos_1);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_1);
            bottom.Base_Pivot.setPosition(0.12);
        }

        //Destacker position 2
        if (gamepad1.right_bumper) {
            bottom.Destacker_Left.setPosition(setpoints.De_Pos_2);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_2);
        }

        //Destacker position 3
        if (gamepad1.y) {
            bottom.Destacker_Left.setPosition(setpoints.De_Pos_3);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_3);
        }

        //Destacker position 4
        if (gamepad1.b) {
            bottom.Destacker_Left.setPosition(setpoints.De_Pos_4);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_4);
        }

        //Destacker position 5
        if (gamepad1.a) {
            bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);
        }

        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
            B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad1.back || gamepad2.back){
            CollectIntoBot();
        }

        //Toggle Base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
            B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        if(gamepad1.start || gamepad2.start){
            ConeAlignment();
        }


        //Reduce robot speed
        if(gamepad1.start && constants.slow == 0.6){
            constants.slow = 0.4;
        }else if(gamepad1.start && constants.slow < 0.6){
            constants.slow = 0.6;
        }

        //Sleep
        sleep(10);

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        if(gamepad2.y && top.Top_Gripper.getPosition() == 0){
            TopGripperOpen();
        }else if(gamepad2.y && top.Top_Gripper.getPosition() > 0){
            TopGripperClosed();
        }

        //toggle base gripper
        if(gamepad2.b && bottom.Base_Gripper.getPosition() == 0){
           B_Grip_Open();
        }else if(gamepad2.b && bottom.Base_Gripper.getPosition() > 0){
            B_Grip_Closed();
        }

        //toggle possition of top pivot
        if (gamepad1.x) {
           ToggleTopPivotDeliver();
        }

        if (gamepad2.x) {
            ToggleTopPivotCollect();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //set top pivot position to drop position
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Deliver);
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //toggle position of top gripper
        if(gamepad2.y && top.Top_Gripper.getPosition() == 0){
            TopGripperOpen();
        }else if(gamepad2.y && top.Top_Gripper.getPosition() > 0){
            TopGripperClosed();
        }

        sleep(10);

        //full cycle
        if (gamepad1.left_bumper || gamepad2.left_bumper){
            FullCycle();
        }

        //set slides to High pole
        if(gamepad1.dpad_left || gamepad2.dpad_left){
           DropOffAtHigh();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //toggle base pivot
        if(gamepad2.a && bottom.Base_Pivot.getPosition() != 0.85){
           B_Pivot_Up();
        }else if(gamepad2.a && bottom.Base_Pivot.getPosition() != setpoints.Base_Pivot_Collect ){
           B_Pivot_Down();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right || gamepad2.dpad_right){
           DropOffAtMedium();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //extend slides to collect cone
        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
            CollectAndTopPivotOver();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //bring slides back to bottom
        if(gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0){
            SlidesToZero();
        }

        if(gamepad2.a && bottom.Base_Pivot.getPosition() != 0.85){
            B_Pivot_Up();
        }else if(gamepad2.a && bottom.Base_Pivot.getPosition() != setpoints.Base_Pivot_Collect ){
            B_Pivot_Down();
        }

        //Stop slides if finished running
        if(constants.lowering){
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }

        //stop slides if finished
        CheckVSlidePos();

        FtcDashboard.getInstance().startCameraStream(slide.BackWeb,30);

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        top.init(hardwareMap);

        bottom.init(hardwareMap);

        slide.init(hardwareMap, 2);

    }

    public void ToggleDestackPosition(){
        constants.stakerpos = constants.stakerpos + 1;

        if(constants.stakerpos == 1){
            constants.Destack_position = setpoints.De_Pos_1;
            bottom.Base_Pivot.setPosition(0.12);
        } else if(constants.stakerpos == 2) {
            constants.Destack_position = setpoints.De_Pos_2;
            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
        } else if(constants.stakerpos == 3) {
            constants.Destack_position = setpoints.De_Pos_3;
            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
        } else if(constants.stakerpos == 4) {
            constants.Destack_position = setpoints.De_Pos_4;
            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
        } else if(constants.stakerpos == 5) {
            constants.Destack_position = setpoints.De_Pos_5;
            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
        }
        if(constants.stakerpos > 5){
            constants.stakerpos = 0;
        }
        bottom.Destacker_Left.setPosition(constants.Destack_position);
        bottom.Destacker_Right.setPosition(constants.Destack_position);
    }

    public void B_Grip_Open(){
        try {
            Thread.sleep(50);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Gripper.setPosition(0.4); //close base gripper if it is open
    }

    public void B_Grip_Closed(){
        try {
            Thread.sleep(50);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        bottom.Base_Gripper.setPosition(0); //close base gripper if it is open
    }

    public void B_Pivot_Down(){
       sleep(100);
        bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
    }

    public void B_Pivot_Up(){
        sleep(100);
        bottom.Base_Pivot.setPosition(0.85);
    }

    public void CollectIntoBot(){

        bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

        drive.stopMotors();

        slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Extend.setPower(-1);

        constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;


        while(!constants.conefound && slide.Extend.getCurrentPosition() > setpoints.Stop_Point){

            CheckVSlidePos();
            slide.Extend.setPower(-1);

            constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

            constants.SlowPoint= slide.sensorRange.getDistance(DistanceUnit.MM) < 200;

            if (constants.SlowPoint){
                slide.Extend.setPower(-0.5);
            }else {
                slide.Extend.setPower(-1);
            }

            sleep(20);

        }

        slide.Extend.setPower(0);

        if(constants.conefound) {

            sleep(50);

            bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Closed);

           sleep(200);

           CheckVSlidePos();

            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

            top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open);

            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Flip);

            if (bottom.Destacker_Left.getPosition() < 0.6){
               sleep(400);
            }

            sleep(125);

            slide.Extend.setTargetPosition(0);
            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (slide.Extend.isBusy()) {

                CheckVSlidePos();

                bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Closed);

                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Flip);

                slide.Extend.setPower(0.8);

            }

            CheckVSlidePos();

            slide.Extend.setPower(0);

            bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
            bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);

            if(bottom.Base_Pivot.getPosition() > 0.7) {

                sleep(200);

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Nest_Position);

                bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Open);

                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Out_Way);

                sleep(250);

                top.Top_Gripper.setPosition(setpoints.Top_Gripper_Closed);

            }

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }else{
            slide.Extend.setTargetPosition(0);
            slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slide.Extend.isBusy()) {
               CheckVSlidePos();
               slide.Extend.setPower(0.8);
            }
            slide.Extend.setPower(0);
        }
    }

    public void ToggleTopPivotDeliver(){
        constants.Toppos2 = constants.Toppos2 + 1;

        if(constants.Toppos2 == 1){
            top.Top_Pivot.setPosition(0.22);
        }else if(constants.Toppos2 == 2){
            top.Top_Pivot.setPosition(0.5);
        }

        if(constants.Toppos2 > 2){
            constants.Toppos2 = 0;
        }
    }

    public void ToggleTopPivotCollect(){
        constants.Toppos = constants.Toppos + 1;

        if(constants.Toppos == 1){
            top.Top_Pivot.setPosition(0.65);
        }else if(constants.Toppos == 2){
            top.Top_Pivot.setPosition(1);
        }

        if(constants.Toppos > 2){
            constants.Toppos = 0;
        }
    }

    public void ConeAlignment(){
        bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Cone_Alignment);
        try {
            Thread.sleep(600);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        constants.Cone_power = 0.2;
        constants.rectPositionFromLeft = slide.Cone_Pipeline_Blue.getRectX();
        while (Math.abs(constants.rectPositionFromLeft - constants.CenterOfScreen) > 25 && (Math.abs(gamepad1.right_stick_x) < 0.1)){

            if(Math.abs(constants.rectPositionFromLeft - constants.CenterOfScreen) > 15 ){
                constants.Cone_power = 0.15;
            }
            constants.rectPositionFromLeft = slide.Cone_Pipeline_Blue.getRectX();

            if (constants.rectPositionFromLeft < constants.CenterOfScreen + 10) {
                drive.RF.setPower(1.3*constants.Cone_power);
                drive.RB.setPower(constants.Cone_power);
                drive.LF.setPower(-1.3*constants.Cone_power);
                drive.LB.setPower(-constants.Cone_power);
            } else if (constants.rectPositionFromLeft > constants.CenterOfScreen - 10) {
                drive.RF.setPower(-1.3*constants.Cone_power);
                drive.RB.setPower(-constants.Cone_power);
                drive.LF.setPower(1.3*constants.Cone_power);
                drive.LB.setPower(constants.Cone_power);
            }

        }
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);
        bottom.Base_Pivot.setPosition(0.05);
    }

    public void AlignToPole (double tolerance) {
        if (Math.abs(constants.rectPositionFromLeft - constants.CenterOfScreen) > tolerance && !(gamepad1.left_stick_x > 0.1) && !(gamepad2.left_stick_x > 0.1)){


            if(Math.abs(constants.rectPositionFromLeft - constants.CenterOfScreen) > tolerance + 10){
                constants.power = 0.10;
            }
            constants.rectPositionFromLeft = slide.Pole.getRectX();

            if (constants.rectPositionFromLeft < constants.CenterOfScreen) {

                drive.RF.setPower(-1.2*constants.power);
                drive.RB.setPower(-constants.power);
                drive.LF.setPower(1.2*constants.power);
                drive.LB.setPower(constants.power);

            } else if (constants.rectPositionFromLeft > constants.CenterOfScreen) {

                drive.RF.setPower(1.2*constants.power);
                drive.RB.setPower(constants.power);
                drive.LF.setPower(-1.2*constants.power);
                drive.LB.setPower(-constants.power);
            }

        }

    }

    public void Pods_Down(){

        slide.Odo_raise.setTargetPosition(0);

        slide.Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.Odo_raise.setPower(0.5);

        while (slide.Odo_raise.isBusy()){}

        slide.Odo_raise.setPower(0);


    }

    public void Pods_Up(){

        slide.Odo_raise.setTargetPosition(105);

        slide.Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.Odo_raise.setPower(-0.5);

        while (slide.Odo_raise.isBusy()){}

        slide.Odo_raise.setPower(0);
    }

    public void TopGripperOpen(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open);
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);
    }

    public void TopGripperClosed(){
        try {
            Thread.sleep(100);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
        top.Top_Gripper.setPosition(0);
        slide.Right_Slide.setPower(0);
        slide.Left_Slide.setPower(0);
    }

    public void sleep(long sleep){
        try {
            Thread.sleep(sleep);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }

    public void FullCycle(){

            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

            drive.stopMotors();

            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Extend.setPower(-1);
        constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

            while(!constants.conefound && slide.Extend.getCurrentPosition() > -900){

                CheckVSlidePos();

                constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

                constants.SlowPoint = slide.sensorRange.getDistance(DistanceUnit.MM) < 200;

                if (constants.SlowPoint){
                    slide.Extend.setPower(-0.5);
                }else {
                    slide.Extend.setPower(-1);
                }

                slide.Extend.setPower(-1);
            }

            slide.Extend.setPower(0);

            if(constants.conefound) {

                sleep(50);

                bottom.Base_Gripper.setPosition(0);

                sleep(100);

                CheckVSlidePos();

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);
                top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open_Wide);

                sleep(125);

                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (slide.Extend.isBusy()) {

                    CheckVSlidePos();

                    bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Flip);

                    slide.Extend.setPower(0.6);

                    if(slide.Extend.getCurrentPosition() > -90){
                        bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Open);
                    }
                    if(slide.Extend.getCurrentPosition() > -20){
                        //open top gripper
                        top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open_Wide);

                        //take top pivot to pick up the cone
                        top.Top_Pivot.setPosition(setpoints.Top_Pivot_Nest_Position);
                    }


                }

                CheckVSlidePos();

                slide.Extend.setPower(0);

                bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
                bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);

                if(bottom.Base_Pivot.getPosition() > 0.6) {

                    sleep(100);

                    bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Open);

                    sleep(100);

                    bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Out_Way);

                    sleep(100);

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Nest_Position);

                    sleep(250);

                    top.Top_Gripper.setPosition(setpoints.Top_Gripper_Closed);

                    sleep(150);

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                    sleep(75);

                    bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
                }

                slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);

                slide.Right_Slide.setTargetPosition(setpoints.Top_Pole);
                slide.Left_Slide.setTargetPosition(setpoints.Top_Pole);
                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(slide.Right_Slide.isBusy() && slide.Left_Slide.isBusy()){

                    slide.Right_Slide.setPower(1);
                    slide.Left_Slide.setPower(1);

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Almost_Drop);

                    if(slide.Right_Slide.getCurrentPosition() > setpoints.Top_Pole - 150){
                        top.Top_Pivot.setPosition(setpoints.Top_Pivot_Deliver);
                    }

                }

                slide.Right_Slide.setPower(constants.reverseSlidesPower);
                slide.Left_Slide.setPower(constants.reverseSlidesPower);

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Just_Dropped);

                slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(400);

                top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open);
                if(top.Top_Gripper.getPosition() == setpoints.Top_Gripper_Open) {

                    slide.Right_Slide.setTargetPosition(0);
                    slide.Left_Slide.setTargetPosition(0);

                    slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                    slide.Right_Slide.setPower(-0.9);
                    slide.Left_Slide.setPower(-0.9);

                    constants.lowering = true;
                }

            }else{

                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slide.Extend.setPower(0.8);

                while (slide.Extend.isBusy()) {
                    CheckVSlidePos();
                    slide.Extend.setPower(0.8);
                }

                slide.Extend.setPower(0);

                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
            }

    }

    public void CollectAndTopPivotOver(){

            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

            drive.stopMotors();

            bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

            slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Extend.setPower(-1);
            constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

            while(!constants.conefound && slide.Extend.getCurrentPosition() > setpoints.Stop_Point){
                CheckVSlidePos();

                constants.conefound = slide.sensorRange.getDistance(DistanceUnit.MM) < 70;

                sleep(10);

                if(bottom.Destacker_Right.getPosition() < 0.5){
                    constants.SlowPoint = slide.sensorRange.getDistance(DistanceUnit.MM) < 250;
                }else{
                    constants.SlowPoint = slide.sensorRange.getDistance(DistanceUnit.MM) < 180;
                }


                if (constants.SlowPoint){
                    slide.Extend.setPower(-0.5);
                }else {
                    slide.Extend.setPower(-1);
                }

            }

            slide.Extend.setPower(0);

            if(constants.conefound) {

                sleep(50);

                bottom.Base_Gripper.setPosition(setpoints.Base_Pivot_Collect);

                sleep(100);

                CheckVSlidePos();

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open_Wide);

                sleep(125);

                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Flip);

                if (bottom.Destacker_Left.getPosition() < setpoints.De_Pos_5){
                    sleep(400);
                }

                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (slide.Extend.isBusy()) {

                    CheckVSlidePos();

                    bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Closed);

                    bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Flip);

                    top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

                    slide.Extend.setPower(0.8);
                }

                CheckVSlidePos();

                slide.Extend.setPower(0);

                bottom.Destacker_Left.setPosition(setpoints.De_Pos_5);
                bottom.Destacker_Right.setPosition(setpoints.De_Pos_5);

                sleep(100);

                bottom.Base_Gripper.setPosition(setpoints.Base_Gripper_Open);

                sleep(100);

                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Out_Way);

                sleep(100);

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_Nest_Position);

                sleep(400);

                top.Top_Gripper.setPosition(setpoints.Top_Gripper_Closed);

                sleep(75);

                top.Top_Pivot.setPosition(setpoints.Top_Pivot_After_Delivery);

                sleep(150);

                constants.conefoundcycle = slide.colour.blue() > 2000;

                if (!constants.conefoundcycle){

                    bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);

                    sleep(200);
                }

                slide.Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                slide.Extend.setTargetPosition(0);
                slide.Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (slide.Extend.isBusy()) {
                   CheckVSlidePos();
                    slide.Extend.setPower(0.8);
                }
                slide.Extend.setPower(0);
                bottom.Base_Pivot.setPosition(setpoints.Base_Pivot_Collect);
            }

    }

    public void DropOffAtHigh(){

        slide.Right_Slide.setTargetPosition(1900);
        slide.Left_Slide.setTargetPosition(1900);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.DriveEncoders();
        constants.power = 0.2;

        while(slide.Right_Slide.getCurrentPosition() < 1900 && slide.Left_Slide.getCurrentPosition() < 1900){
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);

            constants.rectPositionFromLeft = slide.Pole.getRectX();

            if (constants.PoleAlignmnet){
                AlignToPole(15);
            }

            drive.DriveEncoders();
        }
        constants.rectPositionFromLeft = slide.Pole.getRectX();
        slide.Right_Slide.setPower(0.005);
        slide.Left_Slide.setPower(0.005);
        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        top.Top_Pivot.setPosition(0.4);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DropOffAtMedium(){

        slide.Right_Slide.setTargetPosition(setpoints.Medium_Pole);
        slide.Left_Slide.setTargetPosition(setpoints.Medium_Pole);
        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.DriveEncoders();

        while(slide.Right_Slide.getCurrentPosition() < 900 && slide.Left_Slide.getCurrentPosition() < 900){
            slide.Right_Slide.setPower(1);
            slide.Left_Slide.setPower(1);

            constants.rectPositionFromLeft = slide.Pole.getRectX();
            constants.power = 0.27;

            if (constants.PoleAlignmnet){
                AlignToPole(15);
            }

        }
        slide.Right_Slide.setPower(constants.reverseSlidesPower);
        slide.Left_Slide.setPower(constants.reverseSlidesPower);

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

        top.Top_Pivot.setPosition(setpoints.Top_Pivot_After_Delivery);

        slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SlidesToZero(){
        top.Top_Gripper.setPosition(setpoints.Top_Gripper_Open);
        if(top.Top_Gripper.getPosition() == setpoints.Top_Gripper_Open) {

            slide.Right_Slide.setTargetPosition(0);
            slide.Left_Slide.setTargetPosition(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            top.Top_Pivot.setPosition(setpoints.Top_Pivot_Waiting_For_Cone);

            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);

            constants.lowering = true;
        }
    }

    public void CheckVSlidePos() {
        if (slide.Right_Slide.getCurrentPosition() < 10 && !slide.Right_Slide.isBusy() && slide.Left_Slide.getCurrentPosition() < 10 && !slide.Left_Slide.isBusy()) {
            slide.Right_Slide.setPower(0);
            slide.Left_Slide.setPower(0);

            slide.Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slide.Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            constants.lowering = false;
        } else if (constants.lowering) {
            slide.Right_Slide.setPower(-0.9);
            slide.Left_Slide.setPower(-0.9);
        }
    }

}

