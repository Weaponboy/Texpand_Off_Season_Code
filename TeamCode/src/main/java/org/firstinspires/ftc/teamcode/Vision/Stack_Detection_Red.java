package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
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

@Autonomous
@Disabled
public class Stack_Detection_Red extends LinearOpMode {

    Stack_Pos_Red thresholdPipe = new Stack_Pos_Red();

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

        Texpandcamera.closeCameraDevice();

    }

}

class Stack_Pos_Red extends OpenCvPipeline {

    Drivetrain drive = new Drivetrain();

    private static boolean L = false;
    private static boolean R = false;
    private static boolean M = false;

    Mat workingmatrix = new Mat();

    static final Rect Left = new Rect(new Point(0, 0), new Point(215, 480));
    static final Rect Middle = new Rect(new Point(0, 215), new Point(430, 480));
    static final Rect Right = new Rect(new Point(0, 430), new Point(640, 480));

    Telemetry telemetry;
    public Stack_Pos_Red(Telemetry t) {telemetry = t;}

    public Stack_Pos_Red() {}

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);

        //Left
        if (Core.mean(workingmatrix.submat(Left)).val[0] > 136 && Core.mean(workingmatrix.submat(Left)).val[0] < 250){
            telemetry.addData("Colour", "Yellow");
            L = true;
        }
        //Right
        else if(Core.mean(workingmatrix.submat(Right)).val[0] > 136 && Core.mean(workingmatrix.submat(Right)).val[0] < 250){
            telemetry.addData("Colour", "Blue");
            R = true;
        }
        //Middle
        else if(Core.mean(workingmatrix.submat(Middle)).val[0] > 136 && Core.mean(workingmatrix.submat(Middle)).val[0] < 250){
            telemetry.addData("Colour", "Red");
            M = true;
        }

        Scalar Red = new Scalar(319, 100, 100);

        Imgproc.rectangle(input, Right, Red, 10);
        Imgproc.rectangle(input, Left, Red, 10);
        Imgproc.rectangle(input, Middle, Red, 10);



        if (L){
            input.submat(Left);
        }else if(R){
            input.submat(Right);
        }else if(M){
            input.submat(Middle);
        }

        return input;
    }



}
