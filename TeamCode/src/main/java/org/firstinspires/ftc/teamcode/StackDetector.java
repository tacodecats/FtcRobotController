package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class StackDetector extends OpMode {

    //hardware variables
    OpenCvCamera phoneCam = null;
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    //Intake intake = new Intake(hardwareMap);
    //static and other variables
    static double ringCount = 0;
    int rect1X = 0;
    int rect1Y = 0;
    int rect2X = 0;
    int rect2Y = 0;

    final double TICKS_PER_REV = 134.4;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    double TICKS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void init() {
        //hardware maps
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
//stop and reset
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// this offsets the two motors that are facing the opposite direction.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void init_loop(){
        //detecting the starter stack height.

        //tells the phone which detector to use
         phoneCam.setPipeline(new RingDetectingPipeline());
        //starts the stream
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

    }
    @Override
    public void loop() {
        if(ringCount == 1){
            //strafe right.
            strafeDrive(.3,-12.0,12);
            //move forward to deposit wobbly goal
            encoderDrive(.6, 96.0,96.0);
            //strafe to drop it off.
            strafeDrive(.3, 24,-24);
            //code to drop off wobbly goal
            //turn to start picking up second wobbly goal
            encoderDrive(.4,-42,42);
            //driving to a mid point to pick up the wobbly goal
            encoderDrive(.6, 30,30);
            //turn to drive and pickup the wobbly goal
            encoderDrive(.4,-6,6);
            //drive and pickup second wobbly goal
            encoderDrive(.6, 43,43);
            //code to pickup wobbly goal.
            //drive backwards to midpoint
            encoderDrive(.6, -43,-43);
            //turn to square to drop off wobbly goal
            encoderDrive(.4,6,-6);
            //drive to the square
            encoderDrive(.6, -30,-30);
            //drop off second wobbly goal.
            //turn to park
            encoderDrive(.4,7,-7);
            //park
            encoderDrive(1,-24,-24);
            //intake.run(.5);
        }else if(ringCount == 4){
            //drive somewhere else
        }else{
            //drive to a different spot
        }
    }
    public void encoderDrive(double speed, double leftInches, double rightInches){
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        // this gets the position and makes the robot ready to move
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void strafeDrive(double speed, double leftInches, double rightInches){
// this creates the variables that will be calculated
        int newLeftFrontTarget = 0;
        int newRightFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightBackTarget = 0;
// it calculates the distance that the robot has to move when you use the method.
        newLeftFrontTarget = FL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        newRightFrontTarget = FR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);
        newRightBackTarget = BR.getCurrentPosition() + (int)(rightInches * TICKS_PER_INCH);
        newLeftBackTarget = BL.getCurrentPosition() + (int)(leftInches * TICKS_PER_INCH);
        // this gets the position and makes the robot ready to move
        // this flips the diagonals which allows the robot to strafe
        FL.setTargetPosition(newLeftFrontTarget);
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(-newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        FR.setPower(Math.abs(speed));
        FL.setPower(Math.abs(speed));
        BR.setPower(Math.abs(speed));
        BL.setPower(Math.abs(speed));

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
// this stops the run to position.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// resets all the data for the encoders.
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }



    class RingDetectingPipeline extends OpenCvPipeline{

        Mat YCbCr = new Mat();
        Mat outPut = new Mat();
        Mat upperCrop = new Mat();
        Mat lowerCrop = new Mat();


        @Override
        public Mat processFrame(Mat input) {
            //image conversion
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //copying input to output
            input.copyTo(outPut);

            //creating the top rectangle
            Rect rect1 = new Rect(rect1X, rect1Y, 119, 69);


            //creating the bottom rectangle
            Rect rect2 = new Rect(rect2X, rect2Y, 119, 20);

            Scalar rectangleColor = new Scalar(0,0,255);

            //drawing rectangles on screen
            Imgproc.rectangle(outPut, rect1, rectangleColor,2);

            Imgproc.rectangle(outPut, rect2, rectangleColor,2);


            //cropping the image for stack height

            //cropping YCbCr, putting it on lowerCrop mat
            lowerCrop = YCbCr.submat(rect1);
            //cropping YCbCr, putting it on upperCrop mat
            upperCrop = YCbCr.submat(rect2);

            //taking the orange color out, placing on mat
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            //taking the orange color out, placing on mat
            Core.extractChannel(upperCrop, upperCrop, 2);

            //take the raw average data, put it on a Scalar variable
            Scalar lowerAverageOrange = Core.mean(lowerCrop);

            //take the raw average data, put it on a Scalar variable
            Scalar upperAverageOrange = Core.mean(lowerCrop);

            //finally, taking the first value of the average and putting it in a variable
            double finalLowerAverage = lowerAverageOrange.val[0];

            double finalUpperAverage = upperAverageOrange.val[0];

            //comparing average values
            if(finalLowerAverage > 15 && finalLowerAverage < 130 && finalUpperAverage < 130){
                ringCount = 4.0;
            }else if(finalLowerAverage > 10 && finalUpperAverage < 15 && finalLowerAverage > 10 && finalUpperAverage < 15){
                ringCount = 0.0;
            }else{
                ringCount = 1.0;
            }
            //this will be what we are showing on the viewport
            return outPut;
        }
    }
}
