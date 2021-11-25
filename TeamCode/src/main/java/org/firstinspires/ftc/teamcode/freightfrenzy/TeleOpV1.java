package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.DatagramPacket;
import java.net.DatagramSocket;

@TeleOp(name = "TeleOpV1")
public class TeleOpV1 extends LinearOpMode {

    //hardware variables
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;
    DcMotor intakeMotor = null;
    DcMotor armMotor= null;
    DcMotor duckMotor = null;
    Servo boxServo = null;
    double motorPower = 1;

    @Override
    public void runOpMode() {

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



    waitForStart();

        if (opModeIsActive()) {
      // Put run blocks here.
            while (opModeIsActive()) {
             // Put loop blocks here.

                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.5; // Counteract imperfect strafing;
                double rx = gamepad1.right_stick_x;

                FL.setPower(y + x + rx);
                BL.setPower(y - x + rx);
                FR.setPower(y - x - rx);
                BR.setPower(y + x - rx);



                intakeMotor.setPower(gamepad2.right_trigger);


                while(gamepad2.dpad_left) {

                    armMotor.setTargetPosition(300);
                    armMotor.setPower(0.3);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

                if(gamepad2.dpad_right) {

                    armMotor.setTargetPosition(0);
                    armMotor.setPower(0.2);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    boxServo.setPosition(0);
                }

                while(gamepad2.dpad_up) {

                    armMotor.setTargetPosition(475);
                    armMotor.setPower(0.3);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }

                while(gamepad2.dpad_down) {

                    armMotor.setTargetPosition(180);
                    armMotor.setPower(0.3);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                while(gamepad2.left_bumper) {

                    boxServo.setPosition(0.2);
                    intakeMotor.setPower(-0.4);


                }

                intakeMotor.setPower(-gamepad2.left_trigger);

               while (gamepad2.right_bumper) {

                    boxServo.setPosition(0.4);
                }


                if(gamepad2.a){

                    duckMotor.setPower(0);
                }
                if(gamepad2.x){

                    duckMotor.setPower(0.5);
                }

                if(gamepad2.b){

                    duckMotor.setPower(-1);
                }

            }
        }

    }

}
