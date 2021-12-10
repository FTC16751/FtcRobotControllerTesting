package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.IntakeUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is a simple routine to test drive utility class.
 */
@Autonomous(name="Red Alliance delivery and Park in the warehouse", group="Red Autonomous")
public class VisionFreightDeliveryParkInWarehouse_RED extends LinearOpMode {

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
                //useArmPosition = 0;

            }
            runAutonomous();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public void runAutonomous() {
        drive.driveRobotDistanceStrafeLeft(80,0.5);
//raise arm to correct position
        belaArm.raiseToPosition(useArmPosition, 0.5);
        drive.driveRobotDistanceForward(48, 0.4);
        sleep(500);
        //spin the intake to deliver the block
        tyraIntake.setIntake(2);
        sleep(2000);
        tyraIntake.setIntake(0);
        sleep(500);
//go park in warehous
        drive.driveRobotDistanceBackward(10, .4);
        drive.rotateRight90Degrees();
        belaArm.raiseToPosition(0, 0.5);
        drive.driveRobotDistanceStrafeRight(48,.4);
        drive.driveRobotDistanceForward(130, 0.5);


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
