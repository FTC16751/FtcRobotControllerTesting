package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.IntakeUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;
/*
 * This is a simple routine to test drive utility class.
 */
@Autonomous(name="P3_BlueSideVision", group="Linear Opmode")
public class P3_BlueSideVision extends LinearOpMode {

    DriveUtil drive = new DriveUtil();
    SpinnerUtil duckSpin = new SpinnerUtil();
    DeliveryUtil belaArm = new DeliveryUtil();
    IntakeUtil tyraIntake = new IntakeUtil();
    OpenCVUtil vision = new OpenCVUtil();

    private OpenCVUtil.SkystoneDeterminationPipeline pipeline;
    int useArmPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initVision();
        drive.init(hardwareMap);duckSpin.init(hardwareMap);belaArm.init(hardwareMap);tyraIntake.init(hardwareMap);

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
                useArmPosition = 2;
            } else if (duckPosition == "LEFT" ) {
                useArmPosition = 1;
            } else if (duckPosition == "RIGHT") {
                useArmPosition = 3;
            } else {
                useArmPosition = 0;
            }


        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void initVision() {
        //necessary for all auto code that uses vision
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
