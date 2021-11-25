package org.firstinspires.ftc.teamcode.freightfrenzy;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Scalar;


public class OpenCVWebcamDetection extends OpenCvPipeline {

    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect leftDuck = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect rightDuck = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static final Rect midDuck = new Rect(
            new Point(100, 35),
            new Point(160, 75));

    public OpenCVWebcamDetection(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(leftDuck);
        Mat right = mat.submat(rightDuck);
        Mat middle = mat.submat(midDuck);

        double leftGs = Core.sumElems(left).val[0] / leftDuck.area() / 255;
        double rightGs = Core.sumElems(right).val[0] / rightDuck.area() / 255;
        double midGs = Core.sumElems(middle).val[0] / midDuck.area() / 255;

        left.release();
        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Mid raw Value",(int) Core. sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(leftGs * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightGs * 100) + "%");
        telemetry.addData("Mid percentage", Math.round(midGs * 100)+ "%");


        return left;
    }

}
