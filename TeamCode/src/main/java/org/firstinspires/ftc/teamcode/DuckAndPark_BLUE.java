package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;

/*
 * This is a simple routine to test utility classes.
 */
@Autonomous(name="Duck Wheel and Park BLUE", group="Blue Autonomous")
public class DuckAndPark_BLUE extends LinearOpMode {

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
       drive.driveRobotDistanceForward(25, .5);
       drive.driveRobotDistanceStrafeRight(55, .4);
       drive.rotateLeft45Degrees();
       drive.driveRobotDistanceBackward(17,.2);
       duckSpin.SpinClockwise(.4);
        sleep(3000);
        duckSpin.stopSpinner();
        drive.rotateRight45Degrees();
        drive.driveRobotDistanceForward(50,.5);
        drive.driveRobotDistanceStrafeLeft(5,.5);
        /* ****************************************************
        autonomous drive code would end right here
        ******************************************************/

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
