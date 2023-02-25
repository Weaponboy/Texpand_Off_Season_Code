package org.firstinspires.ftc.teamcode.Vision.DoubleCameratests;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;
@TeleOp
@Disabled
public class SwitchableWebcams extends OpMode {

    OpenCvSwitchableWebcam Vision;

    private WebcamName BackWeb;

    private WebcamName frontWeb;

    private ElapsedTime runtime = new ElapsedTime();

    Pole_Pipe Pole;

    Blue_Cone_Pipe Cone;

    private DistanceSensor Back_Distance;

    @Override
    public void init() {
        Pole = new Pole_Pipe();

        Cone = new Blue_Cone_Pipe();

        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

        Back_Distance.resetDeviceConfigurationForOpMode();

        BackWeb = hardwareMap.get(WebcamName.class, "Backcam");

        frontWeb = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());



        Vision = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, BackWeb, frontWeb);

        Vision.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {

                Vision.setPipeline(Pole);
                Vision.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) { }
        });

        Vision.setActiveCamera(BackWeb);

    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up){
            Vision.setActiveCamera(BackWeb);
            Vision.setPipeline(Pole);
        }else if(gamepad1.dpad_down){
            Vision.setActiveCamera(frontWeb);
            Vision.setPipeline(Cone);
        }

        telemetry.addData("Distance", Back_Distance.getDistance(DistanceUnit.MM));
        telemetry.addData("Active Camera:", Vision.getActiveCamera());
        telemetry.addData("Camera FPS:", Vision.getFps());
        telemetry.update();
    }

}
