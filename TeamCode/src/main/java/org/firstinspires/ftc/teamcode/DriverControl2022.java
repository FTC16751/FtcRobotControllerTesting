/* P3 2021-22 season Driver Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;
import org.firstinspires.ftc.teamcode.robot.utilities.RobotUtil;

@TeleOp(name="Driver Control 2022", group="Linear Opmode")
public class DriverControl2022 extends LinearOpMode {
    //drive train variables
    DcMotor left_front_motor;
    DcMotor right_front_motor;
    DcMotor left_rear_motor;
    DcMotor right_rear_motor;
    double LEFT_FRONT_POWER;
    double RIGHT_FRONT_POWER;
    double LEFT_REAR_POWER;
    double RIGHT_REAR_POWER;
    double DRIVE_SPEED;
    double L_HYPOTENUSE;
    double R_HYPOTENUSE;
    DcMotor ShootMotor;
    double shootPower;
    DcMotor BillyArm;
    int armStowPosition;
    int armLevel1Position;
    int armLevel2Position;
    int armLevel3Position;
    int armPosition;

    static final double COUNTS_PER_MOTOR_REV = 2450;
    static final double GEAR_REDUCTION = 1.0;
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;


    Servo intake;

    RevBlinkinLedDriver lights;
    int temp = 1;
    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

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

        ShootMotor = hardwareMap.get(DcMotor.class, "Shoot");
        ShootMotor.setDirection(DcMotor.Direction.REVERSE);
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shootPower = .6;

        //Billy Arm
        BillyArm = hardwareMap.get(DcMotor.class, "billyarm");
        BillyArm.setDirection(DcMotor.Direction.REVERSE);
        int minPosition = 0;
        int maxPosition = (int)(COUNTS_PER_DEGREE * 270);
        armStowPosition = (int)(COUNTS_PER_DEGREE * 35);
        armLevel1Position = (int)(COUNTS_PER_DEGREE * 60);
        armLevel2Position = (int)(COUNTS_PER_DEGREE * 90);
        armLevel3Position = (int)(COUNTS_PER_DEGREE * 240);

        intake = hardwareMap.get(Servo.class, "intake");

        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        BillyArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BillyArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Arm Position reset: ",  "Starting at %7d", BillyArm.getCurrentPosition());

        telemetry.addData("Status", "Initialized.  Press Play.");
        telemetry.update();

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /***************************************************************
         * Game Execution Area
         * Runs continous until "STOP" pressed on Driver Station
         ***************************************************************/
        while (opModeIsActive()) {
            //telemetry
            telemetry.addData("Status: ", "Running.");
            telemetry.addData("Drive Speed: ", "%.2f", DRIVE_SPEED);
            telemetry.update();

            //Set driver speed as a percentage of full (normally set to full)
            if (gamepad1.right_bumper & gamepad1.left_bumper) {
                DRIVE_SPEED = .25;
            } else if (gamepad1.left_bumper) {
                DRIVE_SPEED = .5;
            } else if (gamepad1.right_bumper) {
                DRIVE_SPEED = .75;
            } else {
                DRIVE_SPEED = .80;
            }

            /* This enables a tank drive-like arrangement where the left_stick controls the left
            wheels and the right_stick controls the right wheels uses it x/y values and hypotenuse
            to assign magnitude to the stick_y and stick_x values.  Avoids divide by 0 by checking
            hypotenuse
             */
            L_HYPOTENUSE = Math.sqrt(gamepad1.left_stick_y * gamepad1.left_stick_y +
                    gamepad1.left_stick_x * gamepad1.left_stick_x);
            if (L_HYPOTENUSE == 0) {
                LEFT_FRONT_POWER = 0;
                LEFT_REAR_POWER = 0;
            } else {
                LEFT_FRONT_POWER = -gamepad1.left_stick_y *
                        Math.abs(gamepad1.left_stick_y / L_HYPOTENUSE);
                LEFT_FRONT_POWER += gamepad1.left_stick_x *
                        Math.abs(gamepad1.left_stick_x / L_HYPOTENUSE);
                LEFT_REAR_POWER = -gamepad1.left_stick_y *
                        Math.abs(gamepad1.left_stick_y / L_HYPOTENUSE);
                LEFT_REAR_POWER -= gamepad1.left_stick_x *
                        Math.abs(gamepad1.left_stick_x / L_HYPOTENUSE);
            }
            R_HYPOTENUSE = Math.sqrt(gamepad1.right_stick_y * gamepad1.right_stick_y +
                    gamepad1.right_stick_x * gamepad1.right_stick_x);
            if (R_HYPOTENUSE == 0) {
                RIGHT_FRONT_POWER = 0;
                RIGHT_REAR_POWER = 0;
            } else {
                RIGHT_FRONT_POWER = -gamepad1.right_stick_y *
                        Math.abs(gamepad1.right_stick_y / R_HYPOTENUSE);
                RIGHT_FRONT_POWER += gamepad1.right_stick_x *
                        Math.abs(gamepad1.right_stick_x / R_HYPOTENUSE);
                RIGHT_REAR_POWER = -gamepad1.right_stick_y *
                        Math.abs(gamepad1.right_stick_y / R_HYPOTENUSE);
                RIGHT_REAR_POWER -= gamepad1.right_stick_x *
                        Math.abs(gamepad1.right_stick_x / R_HYPOTENUSE);
            }

            //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
            LEFT_FRONT_POWER = Range.clip(LEFT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
            LEFT_REAR_POWER = Range.clip(LEFT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;
            RIGHT_FRONT_POWER = Range.clip(RIGHT_FRONT_POWER, -1.0, 1.0) * DRIVE_SPEED;
            RIGHT_REAR_POWER = Range.clip(RIGHT_REAR_POWER, -1.0, 1.0) * DRIVE_SPEED;

            // Send calculated power to wheels
            left_front_motor.setPower(LEFT_FRONT_POWER);
            right_front_motor.setPower(RIGHT_FRONT_POWER);
            left_rear_motor.setPower(LEFT_REAR_POWER);
            right_rear_motor.setPower(RIGHT_REAR_POWER);

            //AlexTower
            if(gamepad2.right_bumper) {
                ShootMotor.setPower(shootPower);
            }
            if(gamepad2.left_bumper) {
                ShootMotor.setPower(-shootPower);
            }
            else {
                ShootMotor.setPower(0);
            }


            //stow arm

            if(gamepad2.dpad_left && BillyArm.getCurrentPosition() < maxPosition){
                armPosition = armLevel1Position;
                BillyArm.setTargetPosition(armPosition);
                BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BillyArm.setPower(0.5);
            }
            else if(gamepad2.dpad_up && BillyArm.getCurrentPosition() < maxPosition){
                armPosition = armLevel2Position;
                BillyArm.setTargetPosition(armPosition);
                BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BillyArm.setPower(0.5);

            }
            else if(gamepad2.dpad_right && BillyArm.getCurrentPosition() < maxPosition){
                armPosition = armLevel3Position;
                BillyArm.setTargetPosition(armPosition);
                BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BillyArm.setPower(0.5);

            }
            else if (gamepad2.dpad_down && BillyArm.getCurrentPosition() > minPosition){
                armPosition = armStowPosition;
                BillyArm.setTargetPosition(armPosition);
                BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BillyArm.setPower(0.5);
            }
            else {
                armPosition =(int)(COUNTS_PER_DEGREE * 5);;
                BillyArm.setTargetPosition(armPosition);
                BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BillyArm.setPower(0.1);
            }

            telemetry.addData("Arm Test current position", BillyArm.getCurrentPosition());
            telemetry.addData("minposition ", minPosition);
            telemetry.addData("maxposition", maxPosition);
            // telemetry.update();

            // add code for intake spin
            double IntakePower = 0;
            if (gamepad2.y){
                IntakePower = 1;
            }
            else if (gamepad2.a) {
                IntakePower = -1;
            }
            else IntakePower = .5;
            intake.setPosition(IntakePower);
            telemetry.addData("Intake Power", IntakePower);
            telemetry.addData("Intake Position ", intake.getPosition());
            telemetry.update();

            if(temp == 1){
                resetStartTime();
                temp = 2;
            }

            if (time < 85) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            else if (time >= 85 && time <= 90) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }
            else if (time > 90 && time < 110) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            }
            else if (time >= 110 && time <= 120) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }
            //from 91seconds to 94 seconds
            //(time > 85 && time <= 120)
            else
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } //end OpModeIsActive
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
    }  //end runOpMode

} //end program
