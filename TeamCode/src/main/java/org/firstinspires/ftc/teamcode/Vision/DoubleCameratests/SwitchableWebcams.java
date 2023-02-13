package org.firstinspires.ftc.teamcode.Vision.DoubleCameratests;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

public class SwitchableWebcams extends OpMode {

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

        frontWeb = hardwareMap.get(WebcamName.class, "frontCam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        Vision = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, BackWeb, frontWeb);

        Vision.setPipeline(Pole);

        Vision.setActiveCamera(BackWeb);

        Vision.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                Vision.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) { }
        });
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up){
            Vision.setActiveCamera(BackWeb);
        }else if(gamepad1.dpad_down){
            Vision.setActiveCamera(frontWeb);
        }

        telemetry.addData("Distance", Back_Distance.getDistance(DistanceUnit.MM));
        telemetry.addData("Active Camera:", Vision.getActiveCamera());
        telemetry.update();
    }

}
