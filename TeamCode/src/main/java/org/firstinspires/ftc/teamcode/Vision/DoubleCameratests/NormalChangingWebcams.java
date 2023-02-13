package org.firstinspires.ftc.teamcode.Vision.DoubleCameratests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Blue_Cone_Pipe;
import org.firstinspires.ftc.teamcode.Vision.Cone_Alignment.Pole_Pipe;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class NormalChangingWebcams extends OpMode {

    private OpenCvWebcam BackWeb;

    private OpenCvWebcam frontWeb;
    private DistanceSensor Back_Distance;

    private ElapsedTime runtime = new ElapsedTime();

    Pole_Pipe Pole;

    Blue_Cone_Pipe Cone;

    @Override
    public void init() {
        Pole = new Pole_Pipe();

        Cone = new Blue_Cone_Pipe();

        Back_Distance = hardwareMap.get(DistanceSensor.class, "Back distance");

        Back_Distance.resetDeviceConfigurationForOpMode();

        WebcamName backcam = hardwareMap.get(WebcamName.class, "Backcam");

        WebcamName frontcam = hardwareMap.get(WebcamName.class, "frontCam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        BackWeb = OpenCvCameraFactory.getInstance().createWebcam(backcam, cameraMonitorViewId);

        frontWeb = OpenCvCameraFactory.getInstance().createWebcam(frontcam, cameraMonitorViewId);

        BackWeb.setPipeline(Pole);

        frontWeb.setPipeline(Cone);

        BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                BackWeb.getGainControl().setGain(100);

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

        frontWeb.setPipeline(Cone);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up){
            frontWeb.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {
                    BackWeb.stopStreaming();
                }
            });

            BackWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                    BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                    BackWeb.getGainControl().setGain(100);

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

        }else if(gamepad1.dpad_down){
            BackWeb.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
                @Override
                public void onClose() {
                    BackWeb.stopStreaming();
                }
            });

            frontWeb.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {

                    BackWeb.getExposureControl().setMode(ExposureControl.Mode.Manual);

                    BackWeb.getExposureControl().setExposure(30, TimeUnit.MILLISECONDS);

                    BackWeb.getGainControl().setGain(100);

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

        }

        telemetry.addData("Camera front", frontWeb.getFps());
        telemetry.addData("Camera Back", BackWeb.getFps());
        telemetry.update();
    }
}
