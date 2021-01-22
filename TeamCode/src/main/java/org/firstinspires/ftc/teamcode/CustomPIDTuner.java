/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@Autonomous
@Disabled

public class CustomPIDTuner extends LinearOpMode {

    // Declare OpMode members.
    DcMotor FL = null;
    DcMotor FR = null;
    DcMotor BL = null;
    DcMotor BR = null;

    public static PIDCoefficients PID = new PIDCoefficients(0,0,0);
    FtcDashboard dashboard;
    public static double TARGET_POS = 100;
    ElapsedTime PIDTimer = new ElapsedTime();
    double integral = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FL = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        FR = hardwareMap.get(DcMotor.class, "frontRightMotor");
        BL = hardwareMap.get(DcMotor.class, "backLeftMotor");
        BR = hardwareMap.get(DcMotor.class, "backRightMotor");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //stop and reset
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // this offsets the two motors that are facing the opposite direction.
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                moveRobot(TARGET_POS);
                telemetry.addData("FL Current Position", FL.getCurrentPosition());
                telemetry.addData("FR Current Position", FR.getCurrentPosition());
                telemetry.addData("BL Current Position", BL.getCurrentPosition());
                telemetry.addData("BR Current Position", BR.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    void moveRobot(double targetPosition){
        double errorFL = FL.getCurrentPosition();
        double lastErrorFL = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        while (Math.abs(errorFL) <= 9){
            errorFL = FL.getCurrentPosition() - targetPosition;
            double changeInErrorFL = lastErrorFL - errorFL;
            integral += changeInErrorFL * PIDTimer.time();
            double derivativeFL = changeInErrorFL / PIDTimer.time();
            double P = PID.p * errorFL;
            double I = PID.i * integral;
            double D = PID.d * derivativeFL;
            FL.setPower(P + I + D);
            FR.setPower(P + I + D);
            BL.setPower(P + I + D);
            BR.setPower(P + I + D);
            errorFL = lastErrorFL;
            PIDTimer.reset();
        }
    }
}
