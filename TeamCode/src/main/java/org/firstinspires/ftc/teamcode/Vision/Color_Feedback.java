package org.firstinspires.ftc.teamcode.Vision;

//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.VisionDash;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@Disabled
@Autonomous(name="ColorPicker", group="Autonomous")
public class Color_Feedback extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private OpenCvCamera webcam;
    private OpenCvPipeline pipe = new org.firstinspires.ftc.teamcode.Vision.ColorPicker();

    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        // Start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { webcam.startStreaming(432, 240, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
        // Set the pipeline depending on id
        webcam.setPipeline(pipe);

    }


    @Override
    public void init_loop() {

        telemetry.addData("Y", VisionDash.YCbCrReadout[0]);
        telemetry.addData("Cr", VisionDash.YCbCrReadout[1]);
        telemetry.addData("Cb", VisionDash.YCbCrReadout[2]);

    }


    @Override
    public void start() {

    }


    @Override
    public void loop() {

        loopTelemetry();
    }

    //Telemetry to be displayed during init_loop()

    private void initTelemetry(){
        telemetry.addData("Status", "InitLoop");
        telemetry.update();
    }

    //Telemetry to be displayed during loop()

    private void loopTelemetry(){
        telemetry.addData("Status", "TeleOp Running");
        telemetry.update();
    }
}
