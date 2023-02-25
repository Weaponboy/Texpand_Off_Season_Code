package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Red_Cone_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

import java.util.concurrent.TimeUnit;

@TeleOp
public class DoubleGripperRed extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvWebcam FrontWeb;
    OpenCvWebcam BackWeb;
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

    public boolean conefoundcycle;

    private double power;

    OpenCvSwitchableWebcam Vision = new OpenCvSwitchableWebcam() {
        @Override
        public void setActiveCamera(WebcamName cameraName) {

        }

        @Override
        public WebcamName getActiveCamera() {
            return null;
        }

        @Override
        public WebcamName[] getMembers() {
            return new WebcamName[0];
        }

        @Override
        public int openCameraDevice() {
            return 0;
        }

        @Override
        public void openCameraDeviceAsync(AsyncCameraOpenListener cameraOpenListener) {

        }

        @Override
        public void closeCameraDevice() {

        }

        @Override
        public void closeCameraDeviceAsync(AsyncCameraCloseListener cameraCloseListener) {

        }

        @Override
        public void showFpsMeterOnViewport(boolean show) {

        }

        @Override
        public void pauseViewport() {

        }

        @Override
        public void resumeViewport() {

        }

        @Override
        public void setViewportRenderingPolicy(ViewportRenderingPolicy policy) {

        }

        @Override
        public void setViewportRenderer(ViewportRenderer renderer) {

        }

        @Override
        public void startStreaming(int width, int height) {

        }

        @Override
        public void startStreaming(int width, int height, OpenCvCameraRotation rotation) {

        }

        @Override
        public void stopStreaming() {

        }

        @Override
        public void setPipeline(OpenCvPipeline pipeline) {

        }

        @Override
        public int getFrameCount() {
            return 0;
        }

        @Override
        public float getFps() {
            return 0;
        }

        @Override
        public int getPipelineTimeMs() {
            return 0;
        }

        @Override
        public int getOverheadTimeMs() {
            return 0;
        }

        @Override
        public int getTotalFrameTimeMs() {
            return 0;
        }

        @Override
        public int getCurrentPipelineMaxFps() {
            return 0;
        }

        @Override
        public void startRecordingPipeline(PipelineRecordingParameters parameters) {

        }

        @Override
        public void stopRecordingPipeline() {

        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

        }
    };

    public DcMotor Odo_raise  = null;

    private double TotalDist;
    private double AveDist;
    Drivetrain drive = new Drivetrain();

    Gamepad.RumbleEffect customRumbleEffect;

    private ElapsedTime runtime = new ElapsedTime();

    Pole_Pipe Pole;

    double De_Pos_1 = 0.0;
    double De_Pos_2 = 0.12;
    double De_Pos_3 = 0.45;
    double De_Pos_4 = 0.54;
    double De_Pos_5 = 0.72;

    public ColorSensor colour = null;

    public DcMotor Right_Slide = null;
    public DcMotor Left_Slide = null;

    public DcMotor Extend = null;

    public Servo Base_Gripper = null;
    public Servo Base_Pivot = null;
    public Servo Top_Gripper = null;
    public Servo Top_Pivot = null;
    public Servo Destacker_Left = null;
    public Servo Destacker_Right = null;

    private DistanceSensor sensorRange;

    private DistanceSensor Back_Distance;

    private boolean Nest_Occupied = false;

    private double vertical;
    private double horizontal;
    private double pivot;

    private double Base_Pivot_Collect = 0.05;

    private double Base_Pivot_Flip = 0.78;

    private double Base_Pivot_Out_Way = 1;
    Red_Cone_Pipe Cone_Pipeline;
    private double Top_Pivot_Collect = 0.35;

    private double Top_Gripper_Collect_Wide = 0.36;

    private boolean rumble = false;

    private int Toppos = 0;

    private int TopposP = 0;

    private int Toppos2 = 0;
    private int stakerpos = 0;
    private double Destack_position = 0;


    private  double poleDistance = 0;

    private  double TargetPoleDistance = 420;

    private  double TravelDistance = 0;

    private  int Loop = 0;
    private  int Loop2 = 0;

    public double Distance_To_Travel;


    public double ConversionPixelstoCm = 22;//need to tune this

    public double CenterOfScreen = 320;

    public double rectPositionFromLeft = 0;

    private boolean conefound = false;

    private boolean PoleAlignmnet = true;

    private boolean SlowPoint = false;
    private double slow = 0.4;
    private double Cone_power;
    private double slow1 = 0.4;

    private boolean lowering = false;

    @Override
    public void loop() {
        runtime.reset();
        slow1 = (gamepad1.left_trigger * 0.6) + 0.4;

        drive.WithOutEncoders();

        vertical = -gamepad1.right_stick_y;
        horizontal = -gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        RF.setPower(slow1*1.3*(-pivot + (vertical - horizontal)));
        RB.setPower(slow1*(-pivot + (vertical + horizontal)));
        LF.setPower(slow1*1.3*(pivot + (vertical + horizontal)));
        LB.setPower(slow1*(pivot + (vertical - horizontal)));

        if (gamepad1.right_bumper && gamepad2.right_bumper){

            TopposP = TopposP + 1;

            if(TopposP == 1){
                PoleAlignmnet = false;
            }else if(TopposP == 3){
                PoleAlignmnet = true;
            }

            if(TopposP > 2){
                Toppos = 0;
            }

        }


        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed

        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        if (gamepad1.dpad_up) {

        }
        if (gamepad1.right_bumper) {
            Destacker_Left.setPosition(De_Pos_2);
            Destacker_Right.setPosition(De_Pos_2);
            Base_Pivot.setPosition(0.12);
        }
        if (gamepad1.y) {
            Destacker_Left.setPosition(De_Pos_3);
            Destacker_Right.setPosition(De_Pos_3);
            Base_Pivot.setPosition(0.1);
        }
        if (gamepad1.b) {
            Destacker_Left.setPosition(De_Pos_4);
            Destacker_Right.setPosition(De_Pos_4);
            Base_Pivot.setPosition(0.1);
        }
        if (gamepad1.a) {
            Destacker_Left.setPosition(De_Pos_5);
            Destacker_Right.setPosition(De_Pos_5);
            Base_Pivot.setPosition(Base_Pivot_Collect);
        }


        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        if(gamepad2.back){

            Base_Pivot.setPosition(Base_Pivot_Collect);

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);

            conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;


            while(!conefound && Extend.getCurrentPosition() > -900){
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }

                conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

                SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 200;

                if (SlowPoint){
                    Extend.setPower(-0.5);
                }else {
                    Extend.setPower(-1);
                }

                Extend.setPower(-1);
                try {
                    Thread.sleep(20);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                telemetry.addData("Blue:",sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            Extend.setPower(0);

            if(conefound) {
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }

                Top_Pivot.setPosition(0.4);

                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                Base_Pivot.setPosition(Base_Pivot_Flip);

                if (Destacker_Left.getPosition() < 0.6){

                    try {
                        Thread.sleep(400);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                }

                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                if(Base_Gripper.getPosition() == 0){

                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (Extend.isBusy()) {
                        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                            Right_Slide.setPower(0);
                            Left_Slide.setPower(0);

                            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            Right_Slide.setPower(-0.9);
                            Left_Slide.setPower(-0.9);
                        }

                        Base_Gripper.setPosition(0);

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Extend.setPower(0.8);


                    }

                    if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }

                    Extend.setPower(0);

                    Destacker_Left.setPosition(0.8);
                    Destacker_Right.setPosition(0.8);

                    if(Base_Pivot.getPosition() > 0.7) {

                        try {
                            Thread.sleep(200);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(1);

                        Base_Gripper.setPosition(0.4);

                        Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        try {
                            Thread.sleep(250);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Gripper.setPosition(0);
                    }

                }

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }
                    Extend.setPower(0.8);
                }
                Extend.setPower(0);
            }
        }

        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        if(gamepad1.start || gamepad2.start){

            Base_Pivot.setPosition(0.7);
            try {
                Thread.sleep(600);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Cone_power = 0.2;
            rectPositionFromLeft = Cone_Pipeline.getRectX();
            while (Math.abs(rectPositionFromLeft - CenterOfScreen) > 25 && (Math.abs(gamepad1.right_stick_x) < 0.1)){

                if(Math.abs(rectPositionFromLeft - CenterOfScreen) > 15 ){
                    Cone_power = 0.15;
                }
                rectPositionFromLeft = Cone_Pipeline.getRectX();

                if (rectPositionFromLeft < CenterOfScreen + 10) {
                    drive.RF.setPower(1.3*Cone_power);
                    drive.RB.setPower(Cone_power);
                    drive.LF.setPower(-1.3*Cone_power);
                    drive.LB.setPower(-Cone_power);
                } else if (rectPositionFromLeft > CenterOfScreen - 10) {
                    drive.RF.setPower(-1.3*Cone_power);
                    drive.RB.setPower(-Cone_power);
                    drive.LF.setPower(1.3*Cone_power);
                    drive.LB.setPower(Cone_power);
                }

            }
            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);
            Base_Pivot.setPosition(0.05);
        }

        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        if(gamepad2.b && Base_Gripper.getPosition() == 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0.4); //open base gripper if it is closed
        }else if(gamepad2.b && Base_Gripper.getPosition() > 0){
            try {
                Thread.sleep(50);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Gripper.setPosition(0); //close base gripper if it is open
        }

        //toggle possition of top pivot
        if (gamepad1.x) {
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(0.22);
            }else if(Toppos == 3){
                Top_Pivot.setPosition(0.5);
            }
            if(Toppos > 2){
                Toppos = 0;
            }

        }

        if (gamepad2.x) {
            Toppos = Toppos + 1;

            if(Toppos == 1){
                Top_Pivot.setPosition(1);
            }else if(Toppos == 3){
                Top_Pivot.setPosition(0.65);
            }
            if(Toppos > 2){
                Toppos = 0;
            }

        }

        //set top pivot postition
        if(gamepad1.dpad_down || gamepad2.dpad_down){
            Top_Pivot.setPosition(0);
        }

        //toggle positioin of top gripper
        if(gamepad2.y && Top_Gripper.getPosition() == 0){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Top_Gripper.setPosition(Top_Pivot_Collect); //lift up top griper if it is down
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }else if(gamepad2.y && Top_Gripper.getPosition() > 0){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Top_Gripper.setPosition(0); //lower top gripper if it is up
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);
        }

        try {
            Thread.sleep(10);
        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

        if (gamepad2.left_bumper || gamepad1.left_bumper){

                Base_Pivot.setPosition(Base_Pivot_Collect);

                RF.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                LB.setPower(0);

                Base_Pivot.setPosition(Base_Pivot_Collect);

                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Extend.setPower(-1);
                conefound = sensorRange.getDistance(DistanceUnit.MM) < 120;

                while(!conefound && Extend.getCurrentPosition() > -900){
                    if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }
                    conefound = sensorRange.getDistance(DistanceUnit.MM) < 120;

                    SlowPoint = sensorRange.getDistance(DistanceUnit.MM) < 180;

                    if (SlowPoint){
                        Extend.setPower(-0.5);
                    }else {
                        Extend.setPower(-1);
                    }

                    Extend.setPower(-1);

                    telemetry.addData("Blue:", sensorRange.getDistance(DistanceUnit.MM));
                    telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                    telemetry.addData("Cone found:", conefound);
                    telemetry.update();
                }

                Extend.setPower(0);

                if(conefound) {

                    try {
                        Thread.sleep(50);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Base_Gripper.setPosition(0);

                    try {
                        Thread.sleep(100);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }

                    Top_Pivot.setPosition(0.6);

                    Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                    try {
                        Thread.sleep(125);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }



                        Extend.setTargetPosition(0);
                        Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        while (Extend.isBusy()) {
                            if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                                Right_Slide.setPower(0);
                                Left_Slide.setPower(0);

                                Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                lowering = false;
                            }else if(lowering){
                                Right_Slide.setPower(-0.9);
                                Left_Slide.setPower(-0.9);
                            }
                            Base_Pivot.setPosition(0.82);
                            Extend.setPower(0.6);

                            if(Extend.getCurrentPosition() > -90){
                                Base_Gripper.setPosition(0.4);
                            }
                            if(Extend.getCurrentPosition() > -20){
                                //open top gripper
                                Top_Gripper.setPosition(0.35);

                                //take top pivot to pick up the cone
                                Top_Pivot.setPosition(1);
                            }
                        }

                        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                            Right_Slide.setPower(0);
                            Left_Slide.setPower(0);

                            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            Right_Slide.setPower(-0.9);
                            Left_Slide.setPower(-0.9);
                        }
                        Extend.setPower(0);

                        Destacker_Left.setPosition(0.8);
                        Destacker_Right.setPosition(0.8);

                        if(Base_Pivot.getPosition() > 0.6) {

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Base_Gripper.setPosition(0.4);

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Base_Pivot.setPosition(Base_Pivot_Out_Way);

                            try {
                                Thread.sleep(100);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(1);

                            try {
                                Thread.sleep(250);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Gripper.setPosition(0);

                            try {
                                Thread.sleep(150);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }

                            Top_Pivot.setPosition(0.6);

                            try {
                                Thread.sleep(75);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }
                            Base_Pivot.setPosition(Base_Pivot_Collect);
                        }



                    Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    try {
                        Thread.sleep(200);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Right_Slide.setTargetPosition(1800);
                    Left_Slide.setTargetPosition(1800);
                    Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Right_Slide.getCurrentPosition() < 1750 && Left_Slide.getCurrentPosition() < 1750) {
                        Right_Slide.setPower(1);
                        Left_Slide.setPower(1);
                        Top_Pivot.setPosition(0.3);
                        if(Right_Slide.getCurrentPosition() > 1650){
                            Top_Pivot.setPosition(0);
                        }
                    }
                    Right_Slide.setPower(0.005);
                    Left_Slide.setPower(0.005);

                    Top_Pivot.setPosition(0.1);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                    try {
                        Thread.sleep(200);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    Top_Gripper.setPosition(Top_Pivot_Collect);
                    if(Top_Gripper.getPosition() == Top_Pivot_Collect) {

                        Right_Slide.setTargetPosition(0);
                        Left_Slide.setTargetPosition(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        Top_Pivot.setPosition(0.4);

                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);

                        lowering = true;
                    }

                }else{
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    Extend.setPower(0.8);

                    while (Extend.isBusy()) {
                        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                            Right_Slide.setPower(0);
                            Left_Slide.setPower(0);

                            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            Right_Slide.setPower(-0.9);
                            Left_Slide.setPower(-0.9);
                        }
                        Extend.setPower(0.8);
                    }

                    Extend.setPower(0);

                    Base_Pivot.setPosition(Base_Pivot_Collect);
                }



        }

        //Align to the top pole
        if(gamepad2.dpad_left || gamepad1.dpad_left){

            Right_Slide.setTargetPosition(1900);
            Left_Slide.setTargetPosition(1900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.DriveEncoders();
            power = 0.2;
            while(Right_Slide.getCurrentPosition() < 1900 && Left_Slide.getCurrentPosition() < 1900){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();

                if (PoleAlignmnet){
                    AlignToPole(15);
                }

               drive.DriveEncoders();
            }
            rectPositionFromLeft = Pole.getRectX();
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);
            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            Top_Pivot.setPosition(0.4);

//            if(Math.abs(rectPositionFromLeft - CenterOfScreen) > 45) {
//                Top_Pivot.setPosition(0.4);
//            }else{
//                Top_Pivot.setPosition(0);
//            }

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("rect X", Pole.getRectX());
            telemetry.addData("rect Y", Pole.getRectY());
            telemetry.addData("Distance off by ", Math.abs(rectPositionFromLeft - CenterOfScreen));
            telemetry.addData("Power ", power);
            telemetry.update();


        }

        if(gamepad2.a && Base_Pivot.getPosition() != 0.85){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Pivot.setPosition(0.85); //open base gripper if it is closed
        }else if(gamepad2.a && Base_Pivot.getPosition() != Base_Pivot_Collect ){
            try {
                Thread.sleep(100);
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
            Base_Pivot.setPosition(Base_Pivot_Collect); //close base gripper if it is open
        }

        //set slides to Medium pole
        if(gamepad1.dpad_right || gamepad2.dpad_right){

            Loop2 = 0;
            Loop = 0;

            Right_Slide.setTargetPosition(900);
            Left_Slide.setTargetPosition(900);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.DriveEncoders();
            while(Right_Slide.getCurrentPosition() < 900 && Left_Slide.getCurrentPosition() < 900){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);

                rectPositionFromLeft = Pole.getRectX();
                power = 0.27;

               if (PoleAlignmnet){
                    AlignToPole(15);
               }



                telemetry.addData("Distance_To_Travel:", Distance_To_Travel);
                telemetry.update();

            }
            Right_Slide.setPower(0.005);
            Left_Slide.setPower(0.005);

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            Top_Pivot.setPosition(0.4);

//            if(Math.abs(rectPositionFromLeft - CenterOfScreen) > 50) {
//                Top_Pivot.setPosition(0.4);
//            }else{
//                Top_Pivot.setPosition(0);
//            }

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //Stop slides if finished running
        if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }

        //extend slides to collect cone
        if (gamepad2.left_trigger > 0 || gamepad1.back) {


            if(Destacker_Right.getPosition() < 0.5){
                Base_Pivot.setPosition(0.1);
            }else{
                Base_Pivot.setPosition(Base_Pivot_Collect);
            }

            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setPower(-1);
            conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;

            while(!conefound && Extend.getCurrentPosition() > -900){
                if(Right_Slide.getCurrentPosition() < 50 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 50 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }

                conefound = sensorRange.getDistance(DistanceUnit.MM) < 70;
                if(Destacker_Right.getPosition() < 0.5){
                    SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 250;
                }else{
                    SlowPoint= sensorRange.getDistance(DistanceUnit.MM) < 180;
                }


                if (SlowPoint){
                    Extend.setPower(-0.5);
                }else {
                    Extend.setPower(-1);
                }

                try {
                    Thread.sleep(10);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                telemetry.addData("Blue:", sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("motor ticks:", Extend.getCurrentPosition());
                telemetry.addData("Cone found:", conefound);
                telemetry.update();
            }

            Extend.setPower(0);

            if(conefound) {

                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                Base_Gripper.setPosition(0);

                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }

                if(Right_Slide.getCurrentPosition() < 5 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 5 && !Left_Slide.isBusy()){
                    Right_Slide.setPower(0);
                    Left_Slide.setPower(0);

                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lowering = false;
                }else if(lowering){
                    Right_Slide.setPower(-0.9);
                    Left_Slide.setPower(-0.9);
                }


                Top_Gripper.setPosition(Top_Gripper_Collect_Wide);

                try {
                    Thread.sleep(125);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Base_Pivot.setPosition(Base_Pivot_Flip);

                if (Destacker_Left.getPosition() < 0.6){

                    try {
                        Thread.sleep(400);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                }


                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (Extend.isBusy()) {
                        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                            Right_Slide.setPower(0);
                            Left_Slide.setPower(0);

                            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            lowering = false;
                        }else if(lowering){
                            Right_Slide.setPower(-0.9);
                            Left_Slide.setPower(-0.9);
                        }

                        Base_Gripper.setPosition(0);

                        Base_Pivot.setPosition(Base_Pivot_Flip);

                        Top_Pivot.setPosition(0.6);

                        Extend.setPower(0.8);
                    }

                    if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }

                    Extend.setPower(0);

                    Destacker_Left.setPosition(De_Pos_5);
                    Destacker_Right.setPosition(De_Pos_5);

                    if(Base_Pivot.getPosition() > 0.6) {

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Base_Gripper.setPosition(0.4);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }


                        Base_Pivot.setPosition(Base_Pivot_Out_Way);

                        try {
                            Thread.sleep(100);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(1);

                        try {
                            Thread.sleep(400);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Gripper.setPosition(0);

                        try {
                            Thread.sleep(200);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        Top_Pivot.setPosition(0.4);

                        try {
                            Thread.sleep(75);
                        }catch (Exception e){
                            System.out.println(e.getMessage());
                        }

                        conefoundcycle = colour.blue() > 2000;

                        if (!conefoundcycle){
                            Base_Pivot.setPosition(Base_Pivot_Collect);
                            try {
                                Thread.sleep(200);
                            }catch (Exception e){
                                System.out.println(e.getMessage());
                            }
                        }

                    }



                Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }else{
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
                        Right_Slide.setPower(0);
                        Left_Slide.setPower(0);

                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lowering = false;
                    }else if(lowering){
                        Right_Slide.setPower(-0.9);
                        Left_Slide.setPower(-0.9);
                    }
                    Extend.setPower(0.8);
                }
                Extend.setPower(0);
            }
        }

//bring slides back to bottom
        if(gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0){
            Top_Gripper.setPosition(Top_Pivot_Collect);
            if(Top_Gripper.getPosition() == Top_Pivot_Collect) {

                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);

                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Top_Pivot.setPosition(0.6);



                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);

                lowering = true;
            }
        }

//stop slides if finished
        if(Right_Slide.getCurrentPosition() < 10 && !Right_Slide.isBusy() && Left_Slide.getCurrentPosition() < 10 && !Left_Slide.isBusy()){
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowering = false;
        }else if(lowering){
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
        }



        FtcDashboard.getInstance().startCameraStream(BackWeb,30);

        telemetry.addData("Right Stick Y:", gamepad2.right_stick_y);
        telemetry.addData("Right Stick X:", gamepad2.right_stick_x);
        telemetry.addData("Odo ticks:", Odo_raise.getCurrentPosition());
        telemetry.addData("Active Camera:", Vision.getActiveCamera());
        telemetry.addData("Destacker Left:", Destacker_Left.getPosition());
        telemetry.addData("Destacker Right:", Destacker_Right.getPosition());
        telemetry.addData("Base Pivot:", Base_Pivot.getPosition());
        telemetry.addData("Stacker pos:", stakerpos);
        telemetry.addData("Pole Width:", Pole.Rect_Width);
        telemetry.addData("MM range b:", sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("MM range f", Back_Distance.getDistance(DistanceUnit.MM));
        telemetry.addData("Blue:", colour.blue());
        telemetry.addData("RF Power:", RF.getPower());
        telemetry.addData("RB Power:", RB.getPower());
        telemetry.addData("LF Power:", LF.getPower());
        telemetry.addData("LB Power:", LB.getPower());
        telemetry.addData("motor ticks extend:", Extend.getCurrentPosition());
        telemetry.addData("Pole X:", Pole.getRectX());
        telemetry.addData("Cone X", Cone_Pipeline.getRectX());
        telemetry.update();

    }

    //init
    @Override
    public void init() {

        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");

        Odo_raise = hardwareMap.get(DcMotor.class, "Odo_motor");

        Right_Slide = hardwareMap.get(DcMotor.class, "Right slide");
        Left_Slide = hardwareMap.get(DcMotor.class, "Left slide");

        colour = hardwareMap.get(ColorSensor.class, "colour");

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

        Extend = hardwareMap.get(DcMotor.class, "Extend");

        Base_Gripper = hardwareMap.get(Servo.class,"Base_Gripper");
        Base_Pivot = hardwareMap.get(Servo.class,"Base_Pivot");
        Top_Gripper = hardwareMap.get(Servo.class,"Top_Gripper");
        Top_Pivot = hardwareMap.get(Servo.class,"Top_Pivot");
        Destacker_Left = hardwareMap.get(Servo.class,"Destacker Left");
        Destacker_Right = hardwareMap.get(Servo.class,"Destacker Right");

        Destacker_Left.setDirection(Servo.Direction.REVERSE);
        Base_Gripper.setDirection(Servo.Direction.FORWARD);
        Base_Pivot.setDirection(Servo.Direction.FORWARD);
        Top_Gripper.setDirection(Servo.Direction.FORWARD);
        Top_Pivot.setDirection(Servo.Direction.REVERSE);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Extend.setDirection(DcMotorSimple.Direction.REVERSE);

        Odo_raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Odo_raise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);

        Top_Gripper.setPosition(0.3);

        Base_Gripper.setPosition(0.4);

        drive.init(hardwareMap);

        Back_Distance.resetDeviceConfigurationForOpMode();

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .build();

        Pole = new Pole_Pipe();

        Cone_Pipeline = new Red_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        FrontWeb  = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Backcam"), viewportContainerIds[1]);

        FrontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                FrontWeb.setPipeline(Cone_Pipeline);

                FrontWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                FrontWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);

                FrontWeb.getGainControl().setGain(1);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                FrontWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    FrontWeb.getFocusControl().setFocusLength(450);
                }

                FrontWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        FrontWeb.setPipeline(Cone_Pipeline);

        BackWeb.setPipeline(Pole);
        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {


                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(8, TimeUnit.MILLISECONDS);

                BackWeb.getGainControl().setGain(1);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                BackWeb.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    BackWeb.getFocusControl().setFocusLength(450);
                }

                BackWeb.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        BackWeb.setPipeline(Pole);
        FtcDashboard.getInstance().startCameraStream(BackWeb,30);
        // Set the pipeline depending on id


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("H:", Pole.getH());
        telemetry.addData("S:", Pole.getS());
        telemetry.addData("V:", Pole.getV());
        telemetry.addData("Status:", "Initialized");
        telemetry.addData("top pivot:", Top_Pivot.getPosition());
        telemetry.update();
    }

    public void AlignToPole (double tolerance) {
        if (Math.abs(rectPositionFromLeft - CenterOfScreen) > tolerance && !(gamepad1.left_stick_x > 0.1) && !(gamepad2.left_stick_x > 0.1)){

            telemetry.addData("rect X", Pole.getRectX());
            telemetry.addData("rect Y", Pole.getRectY());
            telemetry.addData("Distance off by ", Math.abs(rectPositionFromLeft - CenterOfScreen));
            telemetry.update();

            if(Math.abs(rectPositionFromLeft - CenterOfScreen) > tolerance + 10){
                power = 0.10;
            }
            rectPositionFromLeft = Pole.getRectX();

            if (rectPositionFromLeft < CenterOfScreen) {

                drive.RF.setPower(-1.2*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(1.2*power);
                drive.LB.setPower(power);

            } else if (rectPositionFromLeft > CenterOfScreen) {

                drive.RF.setPower(1.2*power);
                drive.RB.setPower(power);
                drive.LF.setPower(-1.2*power);
                drive.LB.setPower(-power);
            }

        }

    }

    public void Pods_Down(){

        Odo_raise.setTargetPosition(0);

        Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Odo_raise.setPower(0.5);

        while (Odo_raise.isBusy()){}

        Odo_raise.setPower(0);


    }

    public void Pods_Up(){

        Odo_raise.setTargetPosition(105);

        Odo_raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Odo_raise.setPower(-0.5);

        while (Odo_raise.isBusy()){}

        Odo_raise.setPower(0);
    }


}