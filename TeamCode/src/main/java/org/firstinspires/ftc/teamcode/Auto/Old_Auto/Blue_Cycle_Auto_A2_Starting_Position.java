package org.firstinspires.ftc.teamcode.Auto.Old_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Top_gripper;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Bottom_Gripper_Assembly;
import org.firstinspires.ftc.teamcode.Vision.AprilTags.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Blue_Cycle_Auto_A2_Starting_Position extends LinearOpMode {

    //hardware
    public DcMotor RF = null;
    public DcMotor LF = null;
    public DcMotor RB = null;
    public DcMotor LB = null;

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

    private boolean conefound = false;
    private boolean extending = false;
    private double slow = 1;
    private boolean lowering = false;

    // destacker positions
    double De_Pos_1 = 0.37;
    double De_Pos_2 = 0.44;
    double De_Pos_3 = 0.51;
    double De_Pos_4 = 0.64;
    double De_Pos_5 = 0.85;

    //Drivetrain object
    Drivetrain drive = new Drivetrain();
    Slides slide = new Slides();
    Top_gripper top = new Top_gripper();
    Bottom_Gripper_Assembly bot = new Bottom_Gripper_Assembly();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        drive.init(hardwareMap);
        top.init(hardwareMap);
        slide.init(hardwareMap);
        bot.init(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if(tagOfInterest.id == RIGHT){
           //Position 3
            telemetry.addData("Stop Position", "3");
            telemetry.update();

            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_3();

        }else if(tagOfInterest.id == LEFT){
            //Position 1
            telemetry.addData("Stop Position", "1");
            telemetry.update();

            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_1();

        }else if(tagOfInterest.id == MIDDLE){
            //Position 2
            telemetry.addData("Stop Position", "2");
            telemetry.update();

            Drive_To_Destack();

            DropPreLoad();

            Destack_5();

            Drive_To_Pos_2();

        }

    }

    public void Drive_To_Pos_1(){
        drive.TurnToHeading(0);

        drive.StrafeDistance(40, .5);

        drive.DriveDistanceLongReverse(80, .5);
    }

    public void Drive_To_Pos_2(){
        drive.TurnToHeading(0);

        drive.StrafeDistance(40, .5);

        drive.DriveDistanceLongReverse(50, .5);
    }

    public void Drive_To_Pos_3(){
        drive.TurnToHeading(0);

        drive.StrafeDistance(40, .5);

        drive.DriveDistanceLong(40, .5);
    }

    public void Drive_To_Destack(){
        drive.DriveDistanceLong(140, 0.6);

        drive.TurnToHeading(-90);

        drive.StrafeDistance_Left(25, 0.6);

        drive.DriveDistanceLong(20, 0.6);

        drive.TurnToHeading(-104);

        drive.StrafeDistance(14, 0.6);

        drive.DriveDistanceLong(5, 0.6);
    }

    public void DropPreLoad(){
        Top_Pivot.setPosition(0.5);

        //Extend vertical slides and drop cone
        Right_Slide.setTargetPosition(2000);
        Left_Slide.setTargetPosition(2000);
        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Right_Slide.isBusy() && Left_Slide.isBusy()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }
        Right_Slide.setPower(0);
        Left_Slide.setPower(0);

        Top_Pivot.setPosition(0);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        try {
            Thread.sleep(400);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Gripper.setPosition(0.3);

        //TO DO: Insert WHILE loop
        if(Top_Gripper.getPosition() == 0.3) {
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }
            Top_Pivot.setPosition(0.4);
            Right_Slide.setTargetPosition(0);
            Left_Slide.setTargetPosition(0);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Right_Slide.setPower(-0.9);
            Left_Slide.setPower(-0.9);
            lowering = true;
        }
    }

    public void CheckVSlidePos() {
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
    }

    public void Destack(double De_pos) {
        Destacker_Left.setPosition(De_pos);
        Destacker_Right.setPosition(De_pos);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

    }

    public void Destack_5() {

        //cone one
        Destacker_Left.setPosition(De_Pos_1);
        Destacker_Right.setPosition(De_Pos_1);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


        //cone 2
        Destacker_Left.setPosition(De_Pos_2);
        Destacker_Right.setPosition(De_Pos_2);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


        //cone 3
        Destacker_Left.setPosition(De_Pos_3);
        Destacker_Right.setPosition(De_Pos_3);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


        //cone 4
        Destacker_Left.setPosition(De_Pos_4);
        Destacker_Right.setPosition(De_Pos_4);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(0);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        //cone 5
        Destacker_Left.setPosition(De_Pos_5);
        Destacker_Right.setPosition(De_Pos_5);

        try {
            Thread.sleep(100);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        Top_Pivot.setPosition(0.5);
        Base_Pivot.setPosition(0);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Extend.setPower(-1);
        conefound = colour.blue() > 130;

        while(!conefound && Extend.getCurrentPosition() > -1930){
            CheckVSlidePos();
            conefound = colour.blue() > 130;
            Extend.setPower(-1);
        }
        Extend.setPower(0);
        if(conefound) {
            //close gripper
            Base_Gripper.setPosition(0);

            CheckVSlidePos();

            //open top gripper
            Top_Gripper.setPosition(0.3);

            //make sure gripper is closed
            Base_Gripper.setPosition(0);
            try {
                Thread.sleep(100);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            //if the base gripper is closed retract the horizontal slides
            if(Base_Gripper.getPosition() == 0){
                Extend.setTargetPosition(0);
                Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Extend.isBusy()) {
                    CheckVSlidePos();
                    Base_Pivot.setPosition(1);
                    Extend.setPower(1);
                }

                Extend.setPower(0);

                //bring destacker down
                Destacker_Left.setPosition(1);
                Destacker_Right.setPosition(1);

                while (lowering) {
                    CheckVSlidePos();
                }

                //open base gripper
                Base_Gripper.setPosition(0.4);
                try {
                    Thread.sleep(200);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }

                Top_Gripper.setPosition(0.35);

                //take top pivot to pick up the cone
                Top_Pivot.setPosition(1);


                if(Base_Pivot.getPosition() > 0.9) {

                    try {
                        Thread.sleep(200);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //close top gripper
                    Top_Gripper.setPosition(0);

                    try {
                        Thread.sleep(70);
                    } catch (Exception e) {
                        System.out.println(e.getMessage());
                    }

                    //take top pivot over
                    Top_Pivot.setPosition(0.5);

                    try {
                        Thread.sleep(25);
                    }catch (Exception e){
                        System.out.println(e.getMessage());
                    }

                    //put base pivot back to zero
                    Base_Pivot.setPosition(0);

                }

            }

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Base_Pivot.setPosition(0);

            //Extend vertical slides and drop cone
            Right_Slide.setTargetPosition(2000);
            Left_Slide.setTargetPosition(2000);
            Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(Right_Slide.isBusy() && Left_Slide.isBusy()){
                Right_Slide.setPower(1);
                Left_Slide.setPower(1);
            }
            Right_Slide.setPower(0);
            Left_Slide.setPower(0);

            Top_Pivot.setPosition(0);

            Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            try {
                Thread.sleep(200);
            }catch (Exception e){
                System.out.println(e.getMessage());
            }

            Top_Gripper.setPosition(0.3);

            //TO DO: Insert WHILE loop
            if(Top_Gripper.getPosition() == 0.3) {
                try {
                    Thread.sleep(150);
                }catch (Exception e){
                    System.out.println(e.getMessage());
                }
                Top_Pivot.setPosition(0.4);
                Right_Slide.setTargetPosition(0);
                Left_Slide.setTargetPosition(0);
                Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Right_Slide.setPower(-0.9);
                Left_Slide.setPower(-0.9);
                lowering = true;
            }

        }else{
            Top_Pivot.setPosition(0.5);
            Extend.setTargetPosition(0);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Extend.isBusy()) {
                CheckVSlidePos();
                Extend.setPower(0.8);
            }
            Extend.setPower(0);
            Base_Pivot.setPosition(1);
        }
        try {
            Thread.sleep(20);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
