package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.IntakeUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test utility classes.
 */
@Autonomous(name="P3 Class Test", group="Linear Opmode")
@Disabled
public class P3_AutoExample extends LinearOpMode {

    DriveUtil drive = new DriveUtil();
    SpinnerUtil duckSpin = new SpinnerUtil();
    DeliveryUtil belaArm = new DeliveryUtil();
    IntakeUtil tyraIntake = new IntakeUtil();

    OpenCVUtil vision = new OpenCVUtil();
    private OpenCVUtil.SkystoneDeterminationPipeline pipeline;
    int useArmPosition = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initVision();
        drive.init(hardwareMap);duckSpin.init(hardwareMap);belaArm.init(hardwareMap);tyraIntake.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;


        /* ****************************************************
        P3- Your autonomous drive code would start right here
        ******************************************************/
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

        runAutonomous();
        /* ****************************************************
        autonomous drive code would end right here
        ******************************************************/

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

    public void runAutonomous() {

        belaArm.raiseToPosition(1, .3);
        sleep(100);

        drive.driveRobotDistanceForward(25, .5);
        drive.driveRobotDistanceStrafeLeft(98, .5);
        drive.driveRobotDistanceBackward(2, .2);
        duckSpin.SpinCounterClockwise(.5);
        sleep(3000);
        duckSpin.stopSpinner();

        drive.driveRobotDistanceForward(70, .9);
        drive.driveRobotDistanceStrafeRight(20,.5);
        drive.rotateRight90Degrees();

        drive.driveRobotDistanceForward(60,.5);
        belaArm.raiseToPosition(useArmPosition, .5);
        drive.driveRobotDistanceForward(10,.2);

        tyraIntake.setIntake(2);
        sleep(2000);
        tyraIntake.setIntake(0);
        drive.driveRobotDistanceBackward(75,.7);
        drive.driveRobotDistanceStrafeRight(20,1);
        belaArm.raiseToPosition(0, 1.0);
    }
}
