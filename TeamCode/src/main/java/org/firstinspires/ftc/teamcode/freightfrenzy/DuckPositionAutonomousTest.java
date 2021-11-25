

package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "DuckPositionAutonomousTest")
public class DuckPositionAutonomousTest extends LinearOpMode {
    OpenCvCamera webcam;
    String level;
    @Override
    public void runOpMode() throws InterruptedException {

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

                sleep(50);
                break;
            case CENTER:
                level = "middle";
                telemetry.addData("Center", level);
                telemetry.addData("Left",placement.pipeline.avg1);
                telemetry.addData("Center",placement.pipeline.avg2);
                telemetry.addData("Right",placement.pipeline.avg3);
                telemetry.update();
                sleep(50);
                break;
            case RIGHT:
                level = "upper";
                telemetry.addData("Right", level);
                telemetry.addData("Left",placement.pipeline.avg1);
                telemetry.addData("Center",placement.pipeline.avg2);
                telemetry.addData("Right",placement.pipeline.avg3);
                telemetry.update();
                sleep(50);
        }
        webcam.stopStreaming();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                runAuto();
            }
        }
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
        sleep(30000);
    }

}
