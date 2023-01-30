package org.firstinspires.ftc.teamcode.Vision.OpmodesToRunPipelines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Sub_Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Piplines.StackVision.Stack_Pos_Blue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled

public class Cone_Stack_Blue extends LinearOpMode {

    Stack_Pos_Blue thresholdPipe;
    Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {

        thresholdPipe = new Stack_Pos_Blue(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(thresholdPipe);
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
        while(opModeIsActive()){

            thresholdPipe.Get_Pos_L();
            thresholdPipe.Get_Pos_R();
            thresholdPipe.Get_Pos_M();

            if (thresholdPipe.M){
                //What we want
            }else if(thresholdPipe.R){
                //Strafe Left 5cm
                drive.StrafeDistance_Left(5, .5);
            }else if(thresholdPipe.L){
                //Strafe Right 5cm
                drive.StrafeDistance(5, .5);
            }
            telemetry.update();
        }

        Texpandcamera.closeCameraDevice();

    }

}


