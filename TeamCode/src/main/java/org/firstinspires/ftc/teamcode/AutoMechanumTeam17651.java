/**************************************************************************************************
 Team 16751 Autonomous Mechanum Code

 12/13/19  Build:  Initial Code

 **************************************************************************************************/
//Set teamcode package for FTC
package org.firstinspires.ftc.teamcode;

//Bring in TellOp and Linear classes for operations mode
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

//Bring in Utility Classes
//import java.util.Math;
//Bring in hardware
//Bring in gyroscope classes for robot telemetry


//Start, name Tele Op Mode for station driving. Class name must be the same as the filename for
//OnBot Java
@Disabled
@Autonomous(name="Auto Mech Mode", group="Linear Opmode")
public class AutoMechanumTeam17651 extends LinearOpMode
{
    //Declare Variables

    //Drive hardware & variables
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

    //Servo hardware & variables
    private Servo lift_servo;
    private Servo grip_servo;
    double LIFT_POSITION;
    double LIFT_UP;
    double LIFT_DOWN;
    double LIFT_START;
    double LIFT_INCREMENT;
    double GRIP_POSITION;
    double GRIP_CLOSE;
    double GRIP_OPEN;
    double GRIP_START;
    double GRIP_SPEED;
    double GRIP_INCREMENT;
    int SERVO_SLEEP;

    
    //IMU hardware & variables
    BNO055IMU imu;
    Orientation angles;     //for updating telemetry
    Acceleration gravity;

    //set timing variable
    private ElapsedTime runtime = new ElapsedTime();

    //In a subclass, we can override or overload instance methods. Overriding indicates that the
    //subclass is replacing inherited behavior.
    @Override

    //Initialize and run program
    public void runOpMode() {
        //update status on driver station
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // For config file:  Motor Port 0 plug into left/front motort and label "LF_motor"
        //      Motor Port 1 plug into left/front motort and label "RF_motor"
        //      Motor Port 2 plug into left/front motort and label "LR_motor"
        //      Motor Port 3 plug into left/front motort and label "RR_motor"
        left_front_motor  = hardwareMap.get(DcMotor.class, "LF_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "RF_motor");
        left_rear_motor  = hardwareMap.get(DcMotor.class, "LR_motor");
        right_rear_motor = hardwareMap.get(DcMotor.class, "RR_motor");

        // Reverse left motors for driving (so it goes counterclockwise to drive forward)
        left_front_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_rear_motor.setDirection(DcMotor.Direction.REVERSE);
        right_rear_motor.setDirection(DcMotor.Direction.FORWARD);


            
            sleep(2000);


        //initialize IMU and create new IMU Parameters object.
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuParameters.loggingEnabled      = true;
        imuParameters.loggingTag          = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        //confirm ready
        telemetry.addData("Status", "Initialized.  Press Play to Begin.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //start IMU from 0 reference.  i.e. wherever the "front" of the controller is pointing
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        runtime.reset();

        /***************************************************************
         * Game Execution Area
         ***************************************************************/

        //Ensure Power is between -1 and 1, then factor down by DRIVE_SPEED
        LEFT_FRONT_POWER = .25;
        LEFT_REAR_POWER = .25;
        RIGHT_FRONT_POWER = .25;
        RIGHT_REAR_POWER = .25;

        // Send calculated power to wheels
        left_front_motor.setPower(LEFT_FRONT_POWER);
        right_front_motor.setPower(RIGHT_FRONT_POWER);
        left_rear_motor.setPower(LEFT_REAR_POWER);
        right_rear_motor.setPower(RIGHT_REAR_POWER);

        // move for x seconds
        sleep(1600);

        //stop
        LEFT_FRONT_POWER = 0;
        LEFT_REAR_POWER = 0;
        RIGHT_FRONT_POWER = 0;
        RIGHT_REAR_POWER = 0;

        // Send calculated power to wheels
        left_front_motor.setPower(LEFT_FRONT_POWER);
        right_front_motor.setPower(RIGHT_FRONT_POWER);
        left_rear_motor.setPower(LEFT_REAR_POWER);
        right_rear_motor.setPower(RIGHT_REAR_POWER);


    }  //end runOpMode

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        /* telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                }
            });  */
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
