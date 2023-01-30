package org.firstinspires.ftc.teamcode.Vision.Piplines.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class OpenCVPipelinetest extends LinearOpMode {

    Threshold_Pipeline thresholdPipe = new Threshold_Pipeline();

    public boolean Pos_1 = false;
    public boolean Pos_2 = false;
    public boolean Pos_3 = false;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(new Threshold_Pipeline(telemetry));
        Texpandcamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Texpandcamera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {


            }
        });

        waitForStart();

        thresholdPipe.Get_Pos_1();
        thresholdPipe.Get_Pos_2();
        thresholdPipe.Get_Pos_3();

        if (thresholdPipe.Pos_1) {
            telemetry.addData("Colour", "Yellow");
            Pos_1 = true;
        } else if (thresholdPipe.Pos_2) {
            telemetry.addData("Colour", "Blue");
            Pos_2 = true;
        } else if (thresholdPipe.Pos_3) {
            telemetry.addData("Colour", "Red");
            Pos_3 = true;
        }

        Texpandcamera.closeCameraDevice();
    }
    public Boolean Get_Pos_1(){
        return Pos_2;
    }

    public Boolean Get_Pos_2(){
        return Pos_3;
    }

    public Boolean Get_Pos_3(){

        return Pos_1;
    }
}




