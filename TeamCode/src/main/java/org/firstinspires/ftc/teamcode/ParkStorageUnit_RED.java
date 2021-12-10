package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Park Storage Unit RED", group="Red Autonomous")

public class ParkStorageUnit_RED extends LinearOpMode {

    //drive train variables
    private DcMotor left_front_motor;
    private DcMotor right_front_motor;
    private DcMotor left_rear_motor;
    private DcMotor right_rear_motor;
    double LEFT_FRONT_POWER;
    double RIGHT_FRONT_POWER;
    double LEFT_REAR_POWER;
    double RIGHT_REAR_POWER;
    double DRIVE_SPEED;
    double L_HYPOTENUSE;
    double R_HYPOTENUSE;

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 537;  //*2 adjust for mecanum
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //IMU hardware & variables
    //private Gyroscope imu;
    BNO055IMU imu;
    Orientation compass = new Orientation();
    Acceleration gravity;

    //Initialize and run program
    @Override
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the chassis drive motors.
        left_front_motor  = hardwareMap.get(DcMotor.class, "Front_Left");
        right_front_motor = hardwareMap.get(DcMotor.class, "Front_Right");
        left_rear_motor  = hardwareMap.get(DcMotor.class, "Rear_Left");
        right_rear_motor = hardwareMap.get(DcMotor.class, "Rear_Right");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_rear_motor.setDirection(DcMotor.Direction.REVERSE);
        right_rear_motor.setDirection(DcMotor.Direction.FORWARD);

        //Set brake condition to full stop (allso can give DcMotor.ZeroPowerBehavior.FLOAT)
        left_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Drive Speed
        DRIVE_SPEED = 1;    //1=Full Speed


        //initialize IMU and create new IMU Parameters object.
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imuControl");
        imu.initialize(imuParameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Calibrated and Initialized. ", "Press Start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed or 30 seconds passed
         ***************************************************************/
        //sleep(5000);  //Set a pause to start to allow other robot to move
        DRIVE_SPEED = .9;   //set drive speed between 0 and 1.0

        //If you want to move on timing:
        //use driveRobotForward(double targetSpeed); or driveRobotBackward(double targetSpeed);
        //or driveRobotStrafeRight(double targetSpeed); or driveRobotStrafeLeft(double targetSpeed);
        //sleep(TimeInMilliseconds); then stopRobot();

        //If you want to move on distance use either of four:
        //driveRobotDistanceForward(double distanceInCM, double targetSpeed);
        //driveRobotDistanceBackward(double distanceInCM, double targetSpeed);
        //driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed);
        //driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed);

        //driveRobotDistanceForward(20,DRIVE_SPEED);
        driveRobotDistanceStrafeLeft(80, .5);
        //driveRobotDistanceStrafeRight(78, .5);
        driveRobotDistanceBackward(114,.5);
        //driveRobotStrafeRight(.5);
        //sleep(500);
        stopRobot();


    } //end run opmode

    /***
     * Routine that will stop robot by setting all motors to 0
     */
    public void stopRobot() {
        // Send calculated power to wheels
        left_front_motor.setPower(0);
        right_front_motor.setPower(0);
        left_rear_motor.setPower(0);
        right_rear_motor.setPower(0);
    }//StopRobot

    /***
     * Routine that drives forward at a given speed
     * @param targetSpeed
     */
    public void driveRobotForward(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(targetSpeed);
        right_front_motor.setPower(targetSpeed);
        left_rear_motor.setPower(targetSpeed);
        right_rear_motor.setPower(targetSpeed);
    }

    /***
     * Routine that drives Backward at a given speed
     * @param targetSpeed
     */
    public void driveRobotBackward(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(-targetSpeed);
        right_front_motor.setPower(-targetSpeed);
        left_rear_motor.setPower(-targetSpeed);
        right_rear_motor.setPower(-targetSpeed);
    }

    /***
     * Routine that strafes right at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeRight(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(targetSpeed);
        right_front_motor.setPower(-targetSpeed);
        left_rear_motor.setPower(-targetSpeed);
        right_rear_motor.setPower(targetSpeed);
    }

    /***
     * Routine that strafes left at a given speed
     * @param targetSpeed
     */
    public void driveRobotStrafeLeft(double targetSpeed) {
        // Send calculated power to wheels
        left_front_motor.setPower(-targetSpeed);
        right_front_motor.setPower(targetSpeed);
        left_rear_motor.setPower(targetSpeed);
        right_rear_motor.setPower(-targetSpeed);
    }

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceForward(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);

            telemetry.addData("Status: ", "Robot Driving Forward");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance forward

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceBackward(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(-targetSpeed);
            right_front_motor.setPower(-targetSpeed);
            left_rear_motor.setPower(-targetSpeed);
            right_rear_motor.setPower(-targetSpeed);

            telemetry.addData("Status: ", "Robot Driving Backward");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance backward

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_rear_motor.setDirection(DcMotor.Direction.FORWARD);
        right_rear_motor.setDirection(DcMotor.Direction.FORWARD);
        */

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(-targetCount);
        left_rear_motor.setTargetPosition(-targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(-targetSpeed);
            left_rear_motor.setPower(-targetSpeed);
            right_rear_motor.setPower(targetSpeed);

            telemetry.addData("Status: ", "Robot Strafe Right");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance strafe right

    /***
     * Routine to drive a robot forward a certain distance and stop
     * @param distanceInCM
     * @param targetSpeed
     */
    public void driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed) {
        int targetCount;

        //convert sentimeters to number cycles to drive
        //counts_per_rotation/circumference*distane
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * distanceInCM);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(-targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(-targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //conitnue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(-targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(-targetSpeed);

            telemetry.addData("Status: ", "Robot Driving Strafe Left");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance strafe right

    /****************
     * The robot heading is measured counterclockwise from 0 to +180 and clockwise from 0 to -180
     * this function converts to a stanadard 360 degree compass where start = 0 (i.e. North) and
     * the numbers rotate clockwise through 359 back to 0
     * @param headingFromIMU
     * @return converted compass heading as a double
     */
    //compass = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //currentHeading = convertCompass(compass.firstAngle);
    double convertCompass(double headingFromIMU) {
        double newHeading;

        //zero equals zero
        //with negative numbers, clockwise turn by changing negative to positive
        //on the clockwise side, just subtract number from 360
        if(headingFromIMU<=0) newHeading = headingFromIMU * -1;
        else newHeading = 360 - headingFromIMU;

        return newHeading;
    }


}
