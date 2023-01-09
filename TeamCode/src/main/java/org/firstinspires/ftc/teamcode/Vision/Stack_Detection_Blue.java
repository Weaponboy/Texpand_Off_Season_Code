package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class Stack_Detection_Blue extends LinearOpMode {

    Stack_Pos thresholdPipe = new Stack_Pos();


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera Texpandcamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Texpandcamera.setPipeline(new Stack_Pos(telemetry));
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

        //for testing at the moment
        Texpandcamera.closeCameraDevice();

    }

}

class Stack_Pos extends OpenCvPipeline {

    Drivetrain drive = new Drivetrain();

    private boolean L = false;
    private boolean R = false;
    private boolean M = false;

    Mat workingmatrix = new Mat();

    static final Rect Left = new Rect(new Point(175, 150), new Point(275, 400));
    static final Rect Middle = new Rect(new Point(275, 150), new Point(375, 400));
    static final Rect Right = new Rect(new Point(375, 150), new Point(475, 400));

    Telemetry telemetry;
    public Stack_Pos(Telemetry t) {telemetry = t;}

    public Stack_Pos() {}

    @Override
    public Mat processFrame(Mat input) {

        L = false;
        R = false;
        M = false;

        input.copyTo(workingmatrix);

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);
        //Middle
        if(Core.mean(workingmatrix.submat(Middle)).val[0] > 110 && Core.mean(workingmatrix.submat(Middle)).val[0] < 145){
            M = true;
        }
        //Left
        if (Core.mean(workingmatrix.submat(Left)).val[0] > 110 && Core.mean(workingmatrix.submat(Left)).val[0] < 145){
            L = true;
        }
        //Right
        if(Core.mean(workingmatrix.submat(Right)).val[0] > 110 && Core.mean(workingmatrix.submat(Right)).val[0] < 145){
            R = true;
        }


        Scalar blue = new Scalar(319, 100, 100);
        telemetry.addData("Position Left", L);
        telemetry.addData("Position Right", R);
        telemetry.addData("Position Middle", M);
        telemetry.update();

        if (M && !R && !L){
            // What we want
            telemetry.addData("Position", "Yay!!!!!");
            telemetry.update();
        }else if(!M && R && !L){
            //Strafe Left 5cm
            drive.StrafeDistance(-5, 0.5);
            telemetry.addData("Position", "Right");
            telemetry.update();
        }else if(!M && !R && L){
            //Strafe Right 5cm
            drive.StrafeDistance(5, 0.5);
            telemetry.addData("Position", "Left");
            telemetry.update();
        }
        else if(M && R && !L){
            //Strafe Right 2.5cm
            drive.StrafeDistance(-2.5, 0.5);
            telemetry.addData("Position", "Mid Right");
            telemetry.update();
        }
        else if(M && !R && L){
            //Strafe Right 2.5cm
            drive.StrafeDistance(2.5, .5);
            telemetry.addData("Position", "Mid Left");
            telemetry.update();
        }
        telemetry.update();

        Imgproc.rectangle(input, Right, blue, 10);
        Imgproc.rectangle(input, Left, blue, 10);
        Imgproc.rectangle(input, Middle, blue, 10);

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
