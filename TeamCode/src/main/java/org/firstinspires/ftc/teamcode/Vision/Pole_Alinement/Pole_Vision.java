package org.firstinspires.ftc.teamcode.Vision.Pole_Alinement;

//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;


@Autonomous
public class Pole_Vision extends OpMode {
    Drivetrain drive = new Drivetrain();

    private ElapsedTime runtime = new ElapsedTime();
    Pole_Pipe Pole;

    private DistanceSensor Back_Distance;

    private OpenCvCamera webcam;

    private double power;

    private  double poleDistance = 0;

    private  double TargetPoleDistance = 32;

    private  double TravelDistance = 0;

    private  int Loop = 0;



    public double Distance_To_Travel;

    public double ConversionPixelstoCm = 20;//need to tune this

    public double CenterOfScreen = 320;

    public double rectPositionFromLeft = 0;


    public void init() {

        drive.init(hardwareMap);

//        drive.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        drive.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pole = new Pole_Pipe();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Backcam");

        OpenCvWebcam Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(Pole);
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.getExposureControl().setMode(ExposureControl.Mode.Manual);

                Texpandcamera.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                Texpandcamera.getGainControl().setGain(100);

                FocusControl.Mode focusmode = FocusControl.Mode.Fixed;

                Texpandcamera.getFocusControl().setMode(focusmode);

                if (focusmode == FocusControl.Mode.Fixed){
                    Texpandcamera.getFocusControl().setFocusLength(450);
                }

                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { }
        });
        // Set the pipeline depending on id
        Texpandcamera.setPipeline(Pole);

    }


    @Override
    public void init_loop() {
        drive.init(hardwareMap);

//        //Calculate distance to drive to aline
        rectPositionFromLeft = Pole.getRectX();
//
        Distance_To_Travel = rectPositionFromLeft - CenterOfScreen;

        Distance_To_Travel = Distance_To_Travel / 20;

        //Telemetry to be displayed during init_loop()
        telemetry.addData("S", Pole.getS());
        telemetry.addData("V", Pole.getV());
        telemetry.addData("H", Pole.getH());
        telemetry.addData("con", Pole.numcontours());
        telemetry.addData("rects", Pole.getRects());
        telemetry.addData("Target cm", Distance_To_Travel);
        telemetry.addData("Cone Position", Pole.getRectX());
        telemetry.addData("rect X", Pole.getRectX());
        telemetry.addData("rect Y", Pole.getRectY());
        telemetry.addData("Target CM", Distance_To_Travel);
        telemetry.update();
    }



    public void start() {

    }


    @Override
    public void loop() {

        rectPositionFromLeft = Pole.getRectX();
        power = 0.35;
        drive.WithOutEncoders();

        while (rectPositionFromLeft > CenterOfScreen + 10 || rectPositionFromLeft < CenterOfScreen - 10){

            telemetry.addData("rect X", Pole.getRectX());
            telemetry.addData("rect Y", Pole.getRectY());
            telemetry.addData("Target CM", Distance_To_Travel);
            telemetry.update();

            if(rectPositionFromLeft > CenterOfScreen - 5 || rectPositionFromLeft < CenterOfScreen + 5){
                power = 0.27;
            }
            rectPositionFromLeft = Pole.getRectX();

            if (rectPositionFromLeft < CenterOfScreen) {

                drive.RF.setPower(-1.3*power);
                drive.RB.setPower(power);
                drive.LF.setPower(1.3*power);
                drive.LB.setPower(-power);

            } else if (rectPositionFromLeft > CenterOfScreen) {

                drive.RF.setPower(1.3*power);
                drive.RB.setPower(-power);
                drive.LF.setPower(-1.3*power);
                drive.LB.setPower(power);
            }

        }

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);



        //Telemetry to be displayed during loop()
        telemetry.addData("rect X", Pole.getRectX());
        telemetry.addData("rect Y", Pole.getRectY());
        telemetry.addData("Target CM", Distance_To_Travel);
        telemetry.update();
    }

    //Telemetry to be displayed during init_loop()
    private void initTelemetry(){
        telemetry.addData("rect X", Pole.getRectX());
        telemetry.addData("rect Y", Pole.getRectY());
        telemetry.addData("num contours", Pole.numcontours);
        telemetry.addData("num rects", Pole.numrects);
        telemetry.addData("HSV values", Pole.values);
        telemetry.update();
    }

    //Telemetry to be displayed during loop()

    private void loopTelemetry(){

    }
}
