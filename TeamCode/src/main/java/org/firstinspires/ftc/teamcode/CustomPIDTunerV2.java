
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "CustomPIDTunerV2")

public class CustomPIDTunerV2 extends LinearOpMode {

    //hardware variables
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    BNO055IMU imu;
    Orientation angles;

    final double TICKS_PER_REV = 537.6;    // eg: goBILDA Motor Encoder.
    final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 3.779528;     // For figuring circumference
    double TICKS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static PIDCoefficients PID = new PIDCoefficients(1,0,0);
    ElapsedTime PIDTimer = new ElapsedTime();
    double integralFL = 0;
    double integralFR = 0;
    double integralBL = 0;
    double integralBR = 0;

    @Override
    public void runOpMode() {

            //Set IMU for drivetrain to drive straight
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.calibrationDataFile = "BN055IMUCalibration.json";

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            FL = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            FR = hardwareMap.get(DcMotor.class, "frontRightMotor");
            BL = hardwareMap.get(DcMotor.class, "backLeftMotor");
            BR = hardwareMap.get(DcMotor.class, "backRightMotor");

            //stop and reset
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // this offsets the two motors that are facing the opposite direction.
            FL.setDirection(DcMotor.Direction.REVERSE);
            BL.setDirection(DcMotor.Direction.REVERSE);



        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //Set IMU angle orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                moveRobot();
                    }
                }
            }



    public void encoderDriveImu(double speed, double leftInches, double rightInches, float targetAngle, String direction){
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

        if (direction.equals("forward")) {
            while (FL.getCurrentPosition() < newLeftFrontTarget && FR.getCurrentPosition() < newRightFrontTarget && BL.getCurrentPosition() < newLeftBackTarget && BR.getCurrentPosition() < newRightBackTarget){
                //Set IMU angle orientation
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double errorFL = FL.getCurrentPosition();
                double errorFR = FR.getCurrentPosition();
                double errorBL = BL.getCurrentPosition();
                double errorBR = BR.getCurrentPosition();
                double lastErrorFL = 0;
                double lastErrorFR = 0;
                double lastErrorBL = 0;
                double lastErrorBR = 0;
                double Kp = 0;
                double Ki = 0;
                double Kd = 0;
                while (Math.abs(errorFL) <= 9 && Math.abs(errorFR) <= 9 && Math.abs(errorBL) <= 9 && Math.abs(errorBR) <= 9){
                    errorFL = FL.getCurrentPosition() - newLeftFrontTarget;
                    errorFR = FR.getCurrentPosition() - newRightFrontTarget;
                    errorBL = BL.getCurrentPosition() - newLeftBackTarget;
                    errorBR = BR.getCurrentPosition() - newRightBackTarget;
                    double changeInErrorFL = lastErrorFL - errorFL;
                    double changeInErrorFR = lastErrorFR - errorFR;
                    double changeInErrorBL = lastErrorBL - errorBL;
                    double changeInErrorBR = lastErrorBR - errorBR;
                    integralFL += changeInErrorFL * PIDTimer.time();
                    integralFR += changeInErrorFR * PIDTimer.time();
                    integralBL += changeInErrorBL * PIDTimer.time();
                    integralBR += changeInErrorBR * PIDTimer.time();
                    double derivativeFL = changeInErrorFL / PIDTimer.time();
                    double derivativeFR = changeInErrorFR / PIDTimer.time();
                    double derivativeBL = changeInErrorBL / PIDTimer.time();
                    double derivativeBR = changeInErrorBR / PIDTimer.time();
                    double Pfl = PID.p * errorFL;
                    double Pfr = PID.p * errorFR;
                    double Pbl = PID.p * errorBL;
                    double Pbr = PID.p * errorBR;
                    double Ifl = PID.i * integralFL;
                    double Ifr = PID.i * integralFR;
                    double Ibl = PID.i * integralBL;
                    double Ibr = PID.i * integralBR;
                    double Dfl = PID.d * derivativeFL;
                    double Dfr = PID.d * derivativeFR;
                    double Dbl = PID.d * derivativeBL;
                    double Dbr = PID.d * derivativeBR;

                    if (angles.firstAngle < targetAngle){
                        FR.setPower(Math.abs(Pfl + Ifl + Dfl) );
                        FL.setPower(Math.abs(Pfr + Ifr + Dfr) );
                        BR.setPower(Math.abs(Pbl + Ibl + Dbl) );
                        BL.setPower(Math.abs(Pbr + Ibr + Dbr) );
                        errorFL = lastErrorFL;
                        errorFR = lastErrorFR;
                        errorBL = lastErrorBL;
                        errorBR = lastErrorBR;
                        PIDTimer.reset();
                    } else if (angles.firstAngle > targetAngle){
                        FR.setPower(Math.abs(Pfl + Ifl + Dfl) );
                        FL.setPower(Math.abs(Pfr + Ifr + Dfr) );
                        BR.setPower(Math.abs(Pbl + Ibl + Dbl) );
                        BL.setPower(Math.abs(Pbr + Ibr + Dbr) );
                        errorFL = lastErrorFL;
                        errorFR = lastErrorFR;
                        errorBL = lastErrorBL;
                        errorBR = lastErrorBR;
                        PIDTimer.reset();
                    } else {
                        //this gets the absolute speed and converts it into power for the motor.
                        FR.setPower(Math.abs(Pfl + Ifl + Dfl));
                        FL.setPower(Math.abs(Pfr + Ifr + Dfr));
                        BR.setPower(Math.abs(Pbl + Ibl + Dbl));
                        BL.setPower(Math.abs(Pbr + Ibr + Dbr));
                        errorFL = lastErrorFL;
                        errorFR = lastErrorFR;
                        errorBL = lastErrorBL;
                        errorBR = lastErrorBR;
                        PIDTimer.reset();
                    }
                }
            }
        } else {
            while (FL.getCurrentPosition() > newLeftFrontTarget && FR.getCurrentPosition() > newRightFrontTarget && BL.getCurrentPosition() > newLeftBackTarget && BR.getCurrentPosition() > newRightBackTarget){
                //Set IMU angle orientation
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double errorFL = FL.getCurrentPosition();
                double errorFR = FR.getCurrentPosition();
                double errorBL = BL.getCurrentPosition();
                double errorBR = BR.getCurrentPosition();
                double lastErrorFL = 0;
                double lastErrorFR = 0;
                double lastErrorBL = 0;
                double lastErrorBR = 0;
                double Kp = 0;
                double Ki = 0;
                double Kd = 0;
                while (Math.abs(errorFL) <= 9 && Math.abs(errorFR) <= 9 && Math.abs(errorBL) <= 9 && Math.abs(errorBR) <= 9){
                    errorFL = FL.getCurrentPosition() - newLeftFrontTarget;
                    errorFR = FR.getCurrentPosition() - newRightFrontTarget;
                    errorBL = BL.getCurrentPosition() - newLeftBackTarget;
                    errorBR = BR.getCurrentPosition() - newRightBackTarget;
                    double changeInErrorFL = lastErrorFL - errorFL;
                    double changeInErrorFR = lastErrorFR - errorFR;
                    double changeInErrorBL = lastErrorBL - errorBL;
                    double changeInErrorBR = lastErrorBR - errorBR;
                    integralFL += changeInErrorFL * PIDTimer.time();
                    integralFR += changeInErrorFR * PIDTimer.time();
                    integralBL += changeInErrorBL * PIDTimer.time();
                    integralBR += changeInErrorBR * PIDTimer.time();
                    double derivativeFL = changeInErrorFL / PIDTimer.time();
                    double derivativeFR = changeInErrorFR / PIDTimer.time();
                    double derivativeBL = changeInErrorBL / PIDTimer.time();
                    double derivativeBR = changeInErrorBR / PIDTimer.time();
                    double Pfl = PID.p * errorFL;
                    double Pfr = PID.p * errorFR;
                    double Pbl = PID.p * errorBL;
                    double Pbr = PID.p * errorBR;
                    double Ifl = PID.i * integralFL;
                    double Ifr = PID.i * integralFR;
                    double Ibl = PID.i * integralBL;
                    double Ibr = PID.i * integralBR;
                    double Dfl = PID.d * derivativeFL;
                    double Dfr = PID.d * derivativeFR;
                    double Dbl = PID.d * derivativeBL;
                    double Dbr = PID.d * derivativeBR;

                    telemetry.addData("FL:", FL.getCurrentPosition()  );
                    telemetry.addData("FR:", FR.getCurrentPosition()  );
                    telemetry.addData("BL:", BL.getCurrentPosition()  );
                    telemetry.addData("BR:", BR.getCurrentPosition()  );
                    telemetry.addData("FL Target:", FL.getTargetPosition());
                    telemetry.addData("FR Target:", FR.getTargetPosition());
                    telemetry.addData("BL Target:", BL.getTargetPosition());
                    telemetry.addData("BR Target:", BR.getTargetPosition());
                    telemetry.update();

                    if (angles.firstAngle < targetAngle){
                    FR.setPower(Math.abs(Pfl + Ifl + Dfl) - .05);
                    FL.setPower(Math.abs(Pfr + Ifr + Dfr) + .05);
                    BR.setPower(Math.abs(Pbl + Ibl + Dbl) - .05);
                    BL.setPower(Math.abs(Pbr + Ibr + Dbr) + .05);
                    errorFL = lastErrorFL;
                    errorFR = lastErrorFR;
                    errorBL = lastErrorBL;
                    errorBR = lastErrorBR;
                    PIDTimer.reset();
                } else if (angles.firstAngle > targetAngle){
                    FR.setPower(Math.abs(Pfl + Ifl + Dfl) + .05);
                    FL.setPower(Math.abs(Pfr + Ifr + Dfr) - .05);
                    BR.setPower(Math.abs(Pbl + Ibl + Dbl) + .05);
                    BL.setPower(Math.abs(Pbr + Ibr + Dbr) - .05);
                    errorFL = lastErrorFL;
                    errorFR = lastErrorFR;
                    errorBL = lastErrorBL;
                    errorBR = lastErrorBR;
                    PIDTimer.reset();
                } else {
                        //this gets the absolute speed and converts it into power for the motor.
                        FR.setPower(Math.abs(Pfl + Ifl + Dfl));
                        FL.setPower(Math.abs(Pfr + Ifr + Dfr));
                        BR.setPower(Math.abs(Pbl + Ibl + Dbl));
                        BL.setPower(Math.abs(Pbr + Ibr + Dbr));
                        errorFL = lastErrorFL;
                        errorFR = lastErrorFR;
                        errorBL = lastErrorBL;
                        errorBR = lastErrorBR;
                        PIDTimer.reset();
                    }
                }
            }
        }

        while (FR.isBusy() && FL.isBusy() && BR.isBusy() && BL.isBusy()) {
            telemetry.addData("FL:", FL.getCurrentPosition()  );
            telemetry.addData("FR:", FR.getCurrentPosition()  );
            telemetry.addData("BL:", BL.getCurrentPosition()  );
            telemetry.addData("BR:", BR.getCurrentPosition()  );
            telemetry.addData("FL Target:", FL.getTargetPosition());
            telemetry.addData("FR Target:", FR.getTargetPosition());
            telemetry.addData("BL Target:", BL.getTargetPosition());
            telemetry.addData("BR Target:", BR.getTargetPosition());
            telemetry.update();
        }

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

    // Based on the rings, driving to respective box
    public void moveRobot(){
        //Move robot forward
        encoderDriveImu(.5, 30,30, 0, "forward");
        sleep(30000);
    }
}
