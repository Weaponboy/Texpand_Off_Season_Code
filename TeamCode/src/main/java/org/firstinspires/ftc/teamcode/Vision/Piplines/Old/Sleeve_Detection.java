package org.firstinspires.ftc.teamcode.Vision.Piplines.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Autonomous
@Disabled
public class Sleeve_Detection extends LinearOpMode {

    Threshold_Pipeline_2 thresholdPipe = new Threshold_Pipeline_2();

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(new Threshold_Pipeline_2(telemetry));
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

        while (opModeIsActive()) {

        }


        Texpandcamera.closeCameraDevice();
    }
}

class Threshold_Pipeline_2 extends OpenCvPipeline {

    Mat workingmatrix = new Mat();

    private ArrayList<Integer> detections = new ArrayList<>(3);

    static final Rect center = new Rect(new Point(200, 180), new Point(350, 320));

    Telemetry telemetry;

    public Threshold_Pipeline_2(Telemetry t) {
        telemetry = t;
    }

    public Threshold_Pipeline_2() {}

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV);

        if (Core.mean(workingmatrix.submat(center)).val[0] > 30 && Core.mean(workingmatrix.submat(center)).val[0] < 110) {
            telemetry.addData("Colour", "Yellow");
            ArrayList<Integer> pos_1 = null;
            detections = (pos_1);
        } else if (Core.mean(workingmatrix.submat(center)).val[0] > 110 && Core.mean(workingmatrix.submat(center)).val[0] < 135) {
            telemetry.addData("Colour", "Blue");
            ArrayList<Integer> pos_2 = null;
            detections = (pos_2);
        } else if (Core.mean(workingmatrix.submat(center)).val[0] > 136 && Core.mean(workingmatrix.submat(center)).val[0] < 250) {
            telemetry.addData("Colour", "Red");
            ArrayList<Integer> pos_3 = null;
            detections = (pos_3);
        }

        Scalar blue = new Scalar(319, 100, 100);

        Imgproc.rectangle(input, center, blue, 10);

        return workingmatrix;
    }

    public ArrayList<Integer> getLatestDetections()
    {
        return detections;
    }

}