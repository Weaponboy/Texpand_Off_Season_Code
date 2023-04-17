package org.firstinspires.ftc.teamcode.Vision.Cone_Alignment;

//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
@Disabled
public class Cone_Stack_Vision_Red extends OpMode {
    Drivetrain drive = new Drivetrain();

    private ElapsedTime runtime = new ElapsedTime();
    Blue_Cone_Pipe colin;

    private OpenCvCamera webcam;

    public double Distance_To_Travel;

    public double ConversionPixelstoCm = 20;//need to tune this

    public double CenterOfScreen = 640;

    public double rectPositionFromLeft = 0;


    public void init() {

        drive.init(hardwareMap, 1);

//        drive.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colin = new Blue_Cone_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        Texpandcamera.setPipeline(colin);

        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { Texpandcamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
        // Set the pipeline depending on id
        Texpandcamera.setPipeline(colin);

    }


    @Override
    public void init_loop() {

        drive.init(hardwareMap, 1);

        rectPositionFromLeft = colin.getRectX();

        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen;

        Distance_To_Travel = Distance_To_Travel / 20;

        telemetry.addData("Target cm", Distance_To_Travel);
        telemetry.addData("Cone Position", colin.getRectX());
//        telemetry.addData("rect X", colin.getRectX());
//        telemetry.addData("rect Y", colin.getRectY());
//        if (colin.getRectX() > (1280/2) - 40 && colin.getRectX() < (1280/2) + 40){
//            telemetry.addData("Cone", "Middle");
//        }else if(colin.getRectX() < (1280/2) - 40){
//            telemetry.addData("Cone", "Left");
//        } else if (colin.getRectX() > (1280/2) + 40) {
//            telemetry.addData("Cone", "Right");
//        }
        telemetry.update();
    }



    public void start() {

    }


    @Override
    public void loop() {

//        rectPositionFromLeft = colin.rectPositionFromLeft();
//
//        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen;
//
//        Distance_To_Travel = Distance_To_Travel / 20;

        if (Distance_To_Travel > 0){
            drive.StrafeDistance_Left(Distance_To_Travel, 0.5);
            drive.stopMotors();
            Distance_To_Travel = 0;
        }else if (Distance_To_Travel < 0){
            drive.StrafeDistance(-Distance_To_Travel, 0.5);
            drive.stopMotors();
            Distance_To_Travel = 0;
        }

        telemetry.addData("rect X", colin.getRectX());
        telemetry.addData("rect Y", colin.getRectY());
        telemetry.addData("Target CM", Distance_To_Travel);
//        if (colin.getRectX() > (1280/2) - 40 && colin.getRectX() < (1280/2) + 40){
//            telemetry.addData("Cone", "Middle");
//
//        }else if(colin.getRectX() < (1280/2) + 40){
//            telemetry.addData("Cone", "Left");
//
//        } else if (colin.getRectX() > (1280/2) - 40) {
//            telemetry.addData("Cone", "Right");
//
//        }
        telemetry.update();
    }

    //Telemetry to be displayed during init_loop()
    private void initTelemetry(){
        telemetry.addData("rect X", colin.getRectX());
        telemetry.addData("rect Y", colin.getRectY());
        telemetry.addData("num contours", colin.numcontours);
        telemetry.addData("num rects", colin.numrects);
        telemetry.addData("HSV values", colin.values);
        telemetry.update();
    }

    //Telemetry to be displayed during loop()

    private void loopTelemetry(){

    }
}
