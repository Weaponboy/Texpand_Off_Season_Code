package org.firstinspires.ftc.teamcode.Vision;

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

public class Stack_Detection_Blue extends LinearOpMode {

    Stack_Pos thresholdPipe = new Stack_Pos();
    public boolean middle_true = false;
    public boolean right_true = false;
    public boolean left_true = false;


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

    }

}

class Stack_Pos extends OpenCvPipeline {

    Drivetrain drive = new Drivetrain();

    private static boolean L = false;
    private static boolean R = false;
    private static boolean M = false;

    Mat workingmatrix = new Mat();

    static final Rect Left = new Rect(new Point(0, 0), new Point(215, 480));
    static final Rect Middle = new Rect(new Point(0, 215), new Point(430, 480));
    static final Rect Right = new Rect(new Point(0, 430), new Point(640, 480));

    Telemetry telemetry;
    public Stack_Pos(Telemetry t) {telemetry = t;}

    public Stack_Pos() {}

    @Override
    public Mat processFrame(Mat input) {
        input.copyTo(workingmatrix);

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);

        //Left
        if (Core.mean(workingmatrix.submat(Left)).val[0] > 110 && Core.mean(workingmatrix.submat(Left)).val[0] < 135){
            telemetry.addData("Colour", "Yellow");
            L = true;
        }
        //Right
        else if(Core.mean(workingmatrix.submat(Right)).val[0] > 110 && Core.mean(workingmatrix.submat(Right)).val[0] < 135){
            telemetry.addData("Colour", "Blue");
            R = true;
        }
        //Middle
        else if(Core.mean(workingmatrix.submat(Middle)).val[0] > 110 && Core.mean(workingmatrix.submat(Middle)).val[0] < 135){
            telemetry.addData("Colour", "Red");
            M = true;
        }

        Scalar blue = new Scalar(319, 100, 100);

        Imgproc.rectangle(input, Right, blue, 10);
        Imgproc.rectangle(input, Left, blue, 10);
        Imgproc.rectangle(input, Middle, blue, 10);

        if (M && !R && !L){
            // What we want
        }else if(!M && R && !L){
            //Strafe Left 5cm
            drive.StrafeDistance(-5, .5);
        }else if(!M && !R && L){
            //Strafe Right 5cm
            drive.StrafeDistance(5, .5);
        }
        else if(M && R && !L){
            //Strafe Right 2.5cm
            drive.StrafeDistance(-2.5, .5);
        }
        else if(M && !R && L){
            //Strafe Right 2.5cm
            drive.StrafeDistance(2.5, .5);
        }

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
