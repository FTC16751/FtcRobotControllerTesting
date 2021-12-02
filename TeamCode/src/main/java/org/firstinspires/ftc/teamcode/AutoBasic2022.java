/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

//Start, name Tele Op Mode for station driving. Class name must be the same as the filename for
//OnBot Java
@Autonomous(name="Auto Basic 2022 new", group="Linear Opmode")

public class AutoBasic2022 extends LinearOpMode {
    //drive train variables
    private DcMotor left_front_motor;
    private DcMotor right_front_motor;
    private DcMotor left_rear_motor;
    private DcMotor right_rear_motor;
    double DRIVE_SPEED;

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 386.6 * 2;  //*2 adjust for mecanum
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //Arm
    DcMotor ShootMotor;
    double shootPower;
    DcMotor BillyArm;
    int armStowPosition;
    int armLevel1Position;
    int armLevel2Position;
    int armLevel3Position;
    int armPosition;

    static final double ARM_COUNTS_PER_MOTOR_REV = 2450;
    static final double GEAR_REDUCTION = 1.0;
    static final double COUNTS_PER_GEAR_REV = ARM_COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    Servo intake;

    //IMU hardware & variables
    //private Gyroscope imu;
    BNO055IMU imu;
    Orientation compass = new Orientation();
    Acceleration gravity;

    //Vuforia visual variables
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };


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

        //shoot motor
        ShootMotor = hardwareMap.get(DcMotor.class, "Shoot");
        ShootMotor.setDirection(DcMotor.Direction.REVERSE);
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootPower = .6;

        //Billy Arm
        BillyArm = hardwareMap.get(DcMotor.class, "billyarm");
        BillyArm.setDirection(DcMotor.Direction.REVERSE);
        int minPosition = 0;
        int maxPosition = (int)(COUNTS_PER_DEGREE * 270);
        armStowPosition = (int)(COUNTS_PER_DEGREE * 25);
        armLevel1Position = (int)(COUNTS_PER_DEGREE * 50);
        armLevel2Position = (int)(COUNTS_PER_DEGREE * 90);
        armLevel3Position = (int)(COUNTS_PER_DEGREE * 235);

        intake = hardwareMap.get(Servo.class, "intake");

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
        //sleep(5000);
	/*if(lookForDuck()) {
		telemetry.addData("Duck Status. ", "Duck found");
	} else {
        telemetry.addData("Duck Status. ", "No Duck found");
        telemetry.update();
    }
	sleep(10000);*/

        rotateLeft90Degrees();
        sleep(5000);
        rotateRight90Degrees();


        //If you want to move on timing:
        //use driveRobotForward(double targetSpeed); or driveRobotBackward(double targetSpeed);
        //or driveRobotStrafeRight(double targetSpeed); or driveRobotStrafeLeft(double targetSpeed);
        //sleep(TimeInMilliseconds); then stopRobot();

        //If you want to move on distance use either of four:
        //driveRobotDistanceForward(double distanceInCM, double targetSpeed);
        //driveRobotDistanceBackward(double distanceInCM, double targetSpeed);
        //driveRobotDistanceStrafeLeft(double distanceInCM, double targetSpeed);
        //driveRobotDistanceStrafeRight(double distanceInCM, double targetSpeed);

        //use lookForDuck() to return a boolean true if a duck is in the camera view

        driveRobotDistanceForward(20,DRIVE_SPEED);

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

        //revere motors
        reverseMotor(left_front_motor);
        reverseMotor(right_front_motor);
        reverseMotor(left_rear_motor);
        reverseMotor(right_rear_motor);

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

        //return motors
        reverseMotor(left_front_motor);
        reverseMotor(right_front_motor);
        reverseMotor(left_rear_motor);
        reverseMotor(right_rear_motor);

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

        //revere motors
        reverseMotor(right_front_motor);
        reverseMotor(left_rear_motor);

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

            telemetry.addData("Status: ", "Robot Strafe Right");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();

        //return motors
        reverseMotor(right_front_motor);
        reverseMotor(left_rear_motor);

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

        //revere motors
        reverseMotor(left_front_motor);
        reverseMotor(right_rear_motor);

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

            telemetry.addData("Status: ", "Robot Driving Strafe Left");
            telemetry.addData("Distance traveled (cm): ", "%.1f",
                    left_front_motor.getCurrentPosition()/COUNTS_PER_MOTOR_REV*WHEEL_CIRCUMFERENCE);
            telemetry.update();
        }//end while

        stopRobot();

        //return to normal motors
        reverseMotor(left_front_motor);
        reverseMotor(right_rear_motor);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }  //end drive robot distance strafe right

    public void rotateRight90Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 42;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        reverseMotor(right_front_motor);
        reverseMotor(right_rear_motor);

        //set target stop and mode for running to a position
        left_front_motor.setTargetPosition(targetCount);
        right_front_motor.setTargetPosition(targetCount);
        left_rear_motor.setTargetPosition(targetCount);
        right_rear_motor.setTargetPosition(targetCount);

        left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //continue driving until all of the motors hits the distance
        while (left_front_motor.isBusy() || right_front_motor.isBusy() || left_rear_motor.isBusy()
                || right_rear_motor.isBusy()) {
            // Send calculated power to wheels
            left_front_motor.setPower(targetSpeed);
            right_front_motor.setPower(targetSpeed);
            left_rear_motor.setPower(targetSpeed);
            right_rear_motor.setPower(targetSpeed);
        }//end while

        stopRobot();

        //return to normal motors
        reverseMotor(right_front_motor);
        reverseMotor(right_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateLeft90Degrees() {
        int targetCount;
        double targetSpeed = 0.5;
        double diameter = 42;   //diameter in cms measured between left front and right rear or RF and LR

        //convert centimeters to number cycles to drive
        //to make a 90 degree turn, use diameter divide by four; so, diameter * pi / 4
        // counts_per_rotation/circumference*
        targetCount = (int) Math.round(COUNTS_PER_MOTOR_REV / WHEEL_CIRCUMFERENCE * diameter * Math.PI / 4);

        //ensure full stop and reset motors to begin counting movement
        stopRobot();
        sleep(10);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //revere motors
        reverseMotor(left_front_motor);
        reverseMotor(left_rear_motor);

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
        }//end while

        stopRobot();

        //return to normal motors
        reverseMotor(left_front_motor);
        reverseMotor(left_rear_motor);

        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /****************
     * Use this to reverse the motor direction so encoding will work
     * @param thisMotor
     * @return reversed motor
     */
    DcMotor reverseMotor(DcMotor thisMotor) {
        if(thisMotor.getDirection()==DcMotor.Direction.FORWARD) thisMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        else thisMotor.setDirection(DcMotor.Direction.FORWARD);

        return thisMotor;
    }


    /****************
     * The robot heading is measured counterclockwise from 0 to +180 and clockwise from 0 to -180
     * this function converts to a standard 360 degree compass where start = 0 (i.e. North) and
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

    /**
     * Initialize the Vuforia localization engine.
     */
    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
       // vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

}   //end program

