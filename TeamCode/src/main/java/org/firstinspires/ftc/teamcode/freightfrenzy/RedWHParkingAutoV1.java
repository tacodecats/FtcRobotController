

package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "RedWHParkingAutoV1")
public class RedWHParkingAutoV1 extends LinearOpMode {
    OpenCvCamera webcam;
    String level;

    //hardware variables
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    DcMotor intakeMotor = null;
    DcMotor armMotor= null;
    DcMotor duckMotor = null;
    Servo boxServo = null;

    BNO055IMU imu;
    Orientation angles;

    final double TICKS_PER_REV = 537.6;    // eg: goBILDA Motor Encoder.
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.779528;     // For figuring circumference
    double TICKS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        //Set IMU for drivetrain to drive straight
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BN055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FL = hardwareMap.get(DcMotor.class, "FL_drive");
        FR = hardwareMap.get(DcMotor.class, "FR_drive");
        BL = hardwareMap.get(DcMotor.class, "BL_drive");
        BR = hardwareMap.get(DcMotor.class, "BR_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
        boxServo = hardwareMap.get(Servo.class, "boxServo");

        //stop and reset
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // this offsets the two motors that are facing the opposite direction.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        boxServo.setDirection(Servo.Direction.REVERSE);

        // Set drivetrain motors to brake when power is set to 0.
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boxServo.setPosition(0);

        DuckPosition placement = new DuckPosition();
        placement.pipeline = new DuckPosition.SamplePipeline();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(placement.pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        switch (placement.pipeline.getAnalysis()){
            case LEFT:
                level = "lower";
                telemetry.addData("Left", level);
                telemetry.addData("Left",placement.pipeline.avg1);
                telemetry.addData("Center",placement.pipeline.avg2);
                telemetry.addData("Right",placement.pipeline.avg3);
                telemetry.update();

                sleep(1000);
                break;
            case CENTER:
                level = "middle";
                telemetry.addData("Center", level);
                telemetry.addData("Left",placement.pipeline.avg1);
                telemetry.addData("Center",placement.pipeline.avg2);
                telemetry.addData("Right",placement.pipeline.avg3);
                telemetry.update();
                sleep(1000);
                break;
            case RIGHT:
                level = "upper";
                telemetry.addData("Right", level);
                telemetry.addData("Left",placement.pipeline.avg1);
                telemetry.addData("Center",placement.pipeline.avg2);
                telemetry.addData("Right",placement.pipeline.avg3);
                telemetry.update();
                sleep(1000);
        }
        webcam.stopStreaming();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                runAuto();
            }
        }
    }

    // set drive train power to 0
    public void setPowerZero(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    // this stops the run to position.
    public void runUsingEncoders(){
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    // resets all the data for the drive train encoders.
    public void resetEncoders(){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void bottomLvl() {

        armMotor.setTargetPosition(180);
        armMotor.setPower(0.3);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void midLvl() {

        armMotor.setTargetPosition(300);
        armMotor.setPower(0.3);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void highLvl() {

        armMotor.setTargetPosition(500);
        armMotor.setPower(0.3);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void lowMidRelease() {

        boxServo.setPosition(0.2);
        intakeMotor.setPower(-0.4);

    }

    public void highRelease() {

        boxServo.setPosition(0.5);
    }

    public void lowMidPullBack() {

        boxServo.setPosition(-0.2);
    }

    public void highPullBack() {

        boxServo.setPosition(-0.5);
    }

    public void forwardDrive(double speed, double leftInches, double rightInches, float targetAngle){
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
        while (FL.getCurrentPosition() < newLeftFrontTarget && FR.getCurrentPosition() < newRightFrontTarget && BL.getCurrentPosition() < newLeftBackTarget && BR.getCurrentPosition() < newRightBackTarget){
            //Set IMU angle orientation
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (angles.firstAngle < targetAngle){
                FR.setPower(Math.abs(speed) + .05);
                FL.setPower(Math.abs(speed) - .05);
                BR.setPower(Math.abs(speed) + .05);
                BL.setPower(Math.abs(speed) - .05);
            } else if (angles.firstAngle > targetAngle){
                FR.setPower(Math.abs(speed) - .05);
                FL.setPower(Math.abs(speed) + .05);
                BR.setPower(Math.abs(speed) - .05);
                BL.setPower(Math.abs(speed) + .05);
            } else {
                //this gets the absolute speed and converts it into power for the motor.
                FR.setPower(Math.abs(speed));
                FL.setPower(Math.abs(speed));
                BR.setPower(Math.abs(speed));
                BL.setPower(Math.abs(speed));
            }
        }
        // set drive train power to 0
        setPowerZero();
        sleep(250);
        // this stops the run to position.
        runUsingEncoders();
        // resets all the data for the drive train encoders.
        resetEncoders();
    }

    public void backwardDrive(double speed, double leftInches, double rightInches, float targetAngle){
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
        FL.setTargetPosition(-newLeftFrontTarget);
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(-newRightBackTarget);
        BL.setTargetPosition(-newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //this gets the absolute speed and converts it into power for the motor.
        while (FL.getCurrentPosition() > -newLeftFrontTarget && FR.getCurrentPosition() > -newRightFrontTarget && BL.getCurrentPosition() > -newLeftBackTarget && BR.getCurrentPosition() > -newRightBackTarget){
            //Set IMU angle orientation
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Reduces speed of the drive train motors as they approach the end of their path
            if (angles.firstAngle < targetAngle){
                FR.setPower(Math.abs(speed) - .05);
                FL.setPower(Math.abs(speed) + .05);
                BR.setPower(Math.abs(speed) - .05);
                BL.setPower(Math.abs(speed) + .05);
            } else if (angles.firstAngle > targetAngle){
                FR.setPower(Math.abs(speed) + .05);
                FL.setPower(Math.abs(speed) - .05);
                BR.setPower(Math.abs(speed) + .05);
                BL.setPower(Math.abs(speed) - .05);
            } else {
                //this gets the absolute speed and converts it into power for the motor.
                FR.setPower(Math.abs(speed));
                FL.setPower(Math.abs(speed));
                BR.setPower(Math.abs(speed));
                BL.setPower(Math.abs(speed));
            }
        }
        // set drive train power to 0
        setPowerZero();
        sleep(250);
        // this stops the run to position.
        runUsingEncoders();
        // resets all the data for the drive train encoders.
        resetEncoders();
    }

    public void strafeRight(double speed, double leftInches, double rightInches, float targetAngle){
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
        FR.setTargetPosition(-newRightFrontTarget);
        BR.setTargetPosition(newRightBackTarget);
        BL.setTargetPosition(-newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        while (FL.getCurrentPosition() < newLeftFrontTarget && FR.getCurrentPosition() > -newRightFrontTarget && BL.getCurrentPosition() < newLeftBackTarget && BR.getCurrentPosition() > -newRightBackTarget){
            //this gets the absolute speed and converts it into power for the motor.
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
        }
        // set drive train power to 0
        setPowerZero();
        sleep(250);
        // this stops the run to position.
        runUsingEncoders();
        // resets all the data for the drive train encoders.
        resetEncoders();
    }

    public void strafeLeft(double speed, double leftInches, double rightInches, float targetAngle){
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
        FL.setTargetPosition(-newLeftFrontTarget);
        FR.setTargetPosition(newRightFrontTarget);
        BR.setTargetPosition(-newRightBackTarget);
        BL.setTargetPosition(newLeftBackTarget);

        //the robot will run to that position and stop once it gets to the position.
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //this gets the absolute speed and converts it into power for the motor.
        while (FL.getCurrentPosition() > -newLeftFrontTarget && FR.getCurrentPosition() < newRightFrontTarget && BL.getCurrentPosition() > -newLeftBackTarget && BR.getCurrentPosition() < newRightBackTarget){
            //this gets the absolute speed and converts it into power for the motor.
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
        }
        // set drive train power to 0
        setPowerZero();
        sleep(250);
        // this stops the run to position.
        runUsingEncoders();
        // resets all the data for the drive train encoders.
        resetEncoders();
    }


    // Start auto movement code here
    public void runAuto(){
        if (level == "lower"){
            telemetry.addData("Level:", "Lower Level");
            telemetry.update();

        } else if (level == "middle"){
            telemetry.addData("Level:", "Middle Level");
            telemetry.update();
        } else {
            telemetry.addData("Level:", "Upper Level");
            telemetry.update();
        }

        forwardDrive(0.6, 39, 39, 0);
        strafeLeft(0.4,32,32,0);


        sleep(30000);
    }

}
