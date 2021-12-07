package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.utilities.DeliveryUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.SpinnerUtil;

/*
 * This is a simple routine to test utility classes.
 */
@Autonomous(name="RedDeliver and Duck Wheel", group="Linear Opmode")
public class P3_RedDelivery_DuckWheel extends LinearOpMode {

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
       drive.driveRobotDistanceStrafeLeft(100, .4);
       drive.driveRobotDistanceBackward(2,.2);
       duckSpin.SpinCounterClockwise(.4);
        sleep(3000);
        duckSpin.stopSpinner();
        drive.driveRobotDistanceForward(50,.5);
        /* ****************************************************
        autonomous drive code would end right here
        ******************************************************/

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
