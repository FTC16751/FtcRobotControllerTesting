package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;

/*
 * This is a simple routine to test utility classes.
 */
@Autonomous(name="BlueDeliver and Duck Wheel", group="Linear Opmode")
public class P3_BlueDelivery_DuckWheel extends LinearOpMode {

    DriveUtil drive = new DriveUtil();
    SpinnerUtil duckSpin = new SpinnerUtil();
    DeliveryUtil belaArm = new DeliveryUtil();

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive.init(hardwareMap);
        duckSpin.init(hardwareMap);
        belaArm.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        /* ****************************************************
        P3- Your autonomous drive code would start right here
        ******************************************************/
       drive.driveRobotDistanceForward(20, .5);
       drive.driveRobotDistanceStrafeLeft(88, .2);
        duckSpin.SpinCounterClockwise(.5);
        sleep(4000);
        duckSpin.stopSpinner();
        /* drive.driveRobotDistanceBackward(60, 0.5);
        drive.rotateLeft90Degrees();
        duckSpin.SpinCounterClockwise(.5);
        sleep(4000);
        drive.driveRobotDistanceForward(53, 0.5);
        duckSpin.stopSpinner();
       */
        /* ****************************************************
        autonomous drive code would end right here
        ******************************************************/

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
