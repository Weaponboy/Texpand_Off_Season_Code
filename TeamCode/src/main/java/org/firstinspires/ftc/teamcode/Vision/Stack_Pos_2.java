package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Stack_Detection_Blue;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Stack_Pos_2 extends OpenCvPipeline {

    Stack_Detection_Blue open = new Stack_Detection_Blue();

    public boolean L = false;
    public boolean R = false;
    public boolean M = false;


    Mat workingmatrix = new Mat();



    static final Rect Left = new Rect(new Point(175, 150), new Point(275, 400));
    static final Rect Middle = new Rect(new Point(275, 150), new Point(375, 400));
    static final Rect Right = new Rect(new Point(375, 150), new Point(475, 400));

    Telemetry telemetry;

    public Stack_Pos_2(Telemetry t) {
        telemetry = t;
    }

    public Stack_Pos_2() {

    }

    @Override
    public Mat processFrame(Mat input) {

        L = false;
        R = false;
        M = false;

        input.copyTo(workingmatrix);

        Imgproc.cvtColor(workingmatrix, workingmatrix, Imgproc.COLOR_RGB2HSV_FULL);
        //Middle
        if (Core.mean(workingmatrix.submat(Middle)).val[0] > 135 && Core.mean(workingmatrix.submat(Middle)).val[0] < 165) {
            M = true;
        }
        //Left
        if (Core.mean(workingmatrix.submat(Left)).val[0] > 135 && Core.mean(workingmatrix.submat(Left)).val[0] < 165) {
            L = true;
        }
        //Right
        if (Core.mean(workingmatrix.submat(Right)).val[0] > 135 && Core.mean(workingmatrix.submat(Right)).val[0] < 165) {
            R = true;
        }


        Scalar blue = new Scalar(319, 100, 100);
        telemetry.addData("Hue blue left", Core.mean(workingmatrix.submat(Left)).val[0]);
        telemetry.addData("Hue blue right ", Core.mean(workingmatrix.submat(Right)).val[0]);
        telemetry.addData("Hue blue middle", Core.mean(workingmatrix.submat(Middle)).val[0]);
        telemetry.addData("Position Left", L);
        telemetry.addData("Position Right", R);
        telemetry.addData("Position Middle", M);
        telemetry.update();


        Imgproc.rectangle(input, Right, blue, 10);
        Imgproc.rectangle(input, Left, blue, 10);
        Imgproc.rectangle(input, Middle, blue, 10);

        if (L) {
            input.submat(Left);
        } else if (R) {
            input.submat(Right);
        } else if (M) {
            input.submat(Middle);
        }

        return input;
    }

    public Boolean Get_Pos_L(){
        return L;
    }

    public Boolean Get_Pos_R(){
        return R;
    }

    public Boolean Get_Pos_M(){
        return M;
    }
}