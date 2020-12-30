package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "AutonomousV1")

public class AutonomousV1 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //Vuforia key required for tenserflow use. Registered TaCodeCats Vuforia developer license.
    private static final String VUFORIA_KEY =
            "Abb8ztv/////AAABmfkWHjVBIEjck7bzpONXBoOI7PquUpRvN18+exaXjtKfyB8P9fFjTbC7ez8iKXaH8kbsROd88PgV+dbDNHfxvLE/ez8CFf5oRlhjWYQsur1iQjXPbyp3+L8XL5XIjy2Yqh7toBv3GNCeJsxpS7/XefqZA8I1/DxPMfaQsYpkxkC/4J9xTR9m+nkzV3GVJaRdd3xYP6LSqtWV6vBhbBdIm/C37kQd914JGIucoQfgWu9fznniPpKrK8SDRNh9AVz4dLT1QTkuCGQhidpueI8HlRD0T0ROOlgzoIg89KLfJqur//Z+fXSa2zyBw3A/H6691ryAn+rcBUix7S2A1qT6VQwZ4kg106T8BrK+NpqTXQx5";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //hardware variables
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    DcMotor intakeMotor;
    DcMotor shooterMotor;
    DcMotor wobbleArmMotor;
    Servo shooterServo;
    Servo wobbleArmServo;

    final double TICKS_PER_REV = 537.6;    // eg: goBILDA Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.779528;     // For figuring circumference
    double TICKS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);

            FL = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            FR = hardwareMap.get(DcMotor.class, "frontRightMotor");
            BL = hardwareMap.get(DcMotor.class, "backLeftMotor");
            BR = hardwareMap.get(DcMotor.class, "backRightMotor");
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
            shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
            wobbleArmMotor = hardwareMap.get(DcMotor.class, "wobbleArmMotor");
            shooterServo = hardwareMap.get(Servo.class, "shooterServo");
            wobbleArmServo = hardwareMap.get(Servo.class, "wobbleArmServo");

            //stop and reset
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // this offsets the two motors that are facing the opposite direction.
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);

            // Reverse shooter motor so it goes in the correct direction
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Set drivetrain motors to brake when power is set to 0.
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set wobble arm motor encoder to 0.
            wobbleArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set wobble arm motor to use encoder
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //Set servo position to 0
            shooterServo.setPosition(0);
            wobbleArmServo.setPosition(0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0 ) {
                            // empty list.  no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            targetZoneA();
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                // check label to see which target zone to go after.
                                if (recognition.getLabel().equals("Single")) {
                                    telemetry.addData("Target Zone", "B");
                                    targetZoneB();
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target Zone", "C");
                                    targetZoneC();
                                } else {
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                }
                            }
                        }

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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

        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            telemetry.addData("Path", "Running");
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();

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

        while (FR.isBusy() || FL.isBusy() || BR.isBusy() || BL.isBusy()) {
            telemetry.addData("Path", "Running");
            telemetry.update();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();

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

    public void shootingRings() {
        shooterMotor.setPower(-1);
        sleep(4000);
        shooterMotor.setPower(0);
    }

    //Drop wobble goal into place.
    public void dropWobbleGoal() {
        wobbleArmMotor.setTargetPosition(220);
        wobbleArmMotor.setPower(0.5);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        wobbleArmMotor.setPower(0);
        sleep(500);
        wobbleArmServo.setPosition(0.35);
        sleep(500);
        wobbleArmMotor.setTargetPosition(50);
        wobbleArmMotor.setPower(0.5);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        wobbleArmMotor.setPower(0);

    }

// based on the rings, driving to respective box
    public void targetZoneA(){
        //Move forward to box A
        encoderDrive(.8, 78,78);
        dropWobbleGoal();
        sleep(30000);
        //placing wobble goal in square
        strafeDrive(.8,-14,-14);
        encoderDrive(.8,-12,-12);
        shootingRings();

    }
    //Steps for single ring
    public void targetZoneB(){
        //Move forward towards box B
        encoderDrive(.8, 103,103);
        //Strafe left into box B (left and right numbers are negative)
        strafeDrive(.5,-30,-30);
        dropWobbleGoal();
        sleep(30000);
        encoderDrive(.8,-35,-35);
        shootingRings();
    }
    //Steps for four rings
    public void targetZoneC(){
        //Move forward to box C
        encoderDrive(.8, 123,123);
        dropWobbleGoal();
        sleep(30000);
        strafeDrive(.8,-14,-14);
        encoderDrive(.8,-12,-12);
        shootingRings();
    }


}
