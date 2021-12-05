package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test drive utility class.
 */
@Autonomous(name="P3 Vision Auto Test", group="Linear Opmode")
public class P3_AutoVisionExample extends LinearOpMode {

    OpenCVUtil vision = new OpenCVUtil();
    private OpenCVUtil.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        initVision();

        waitForStart();

        if (isStopRequested()) return;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            String duckPosition = String.valueOf(pipeline.getAnalysis());
            telemetry.addData("String", duckPosition);
            telemetry.update();

            if (duckPosition == "CENTER") {
                telemetry.addData("What to do", "Put in middle row!");
                telemetry.update();
            } else if (duckPosition == "LEFT" ) {
                telemetry.addData("What to do", "Put in lowest row!");
                telemetry.update();
            } else if (duckPosition == "RIGHT") {
                telemetry.addData("What to do", "Put in highest row!");
                telemetry.update();
            } else {
                telemetry.addData("What to do", "can't find anything, go park instead");
                telemetry.update();
            }


        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void initVision() {
        vision.init(hardwareMap);

        pipeline = new OpenCVUtil.SkystoneDeterminationPipeline();

        vision.webcam.setPipeline(pipeline);
        vision.webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.

        vision.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                vision.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

}
