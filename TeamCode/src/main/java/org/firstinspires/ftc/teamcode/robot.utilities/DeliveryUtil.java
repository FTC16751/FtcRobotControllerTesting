/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class DeliveryUtil {

    //using GoBilda 5202 motor
    static final double COUNTS_PER_MOTOR_REV = 1425.1;  //*2 adjust for mecanum
    static final double WHEEL_DIAMETER = 9.6;     // In centimeters
    static final double WHEEL_RADIUS = WHEEL_DIAMETER/2; // in cm
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double GEAR_REDUCTION = 2.35; // output (wheel) speed / input (motor) speed
    static final double COUNTS_PER_GEAR_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_GEAR_REV/360;

    //Bela delivery arm Definitions
    DcMotor BillyArm;
    int armPosition;
    Servo intake;
    int minPosition = 0;
    int maxPosition = (int)(COUNTS_PER_DEGREE * 270);;
    int armLevel1Position = (int)(COUNTS_PER_DEGREE * 30);
    int armLevel2Position = (int)(COUNTS_PER_DEGREE * 50);
    int armLevel3Position = (int)(COUNTS_PER_DEGREE * 70);
    int armLevel4Position = (int)(COUNTS_PER_DEGREE * 175);
    //int armLevel3Position = (int)(COUNTS_PER_DEGREE * 235);

    /* local OpMode members. */
    HardwareMap hardwareMap          =  null;

    /* Constructor */
    public DeliveryUtil(){
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        //Billy Arm
        BillyArm = hardwareMap.get(DcMotor.class, "billyarm");
        BillyArm.setDirection(DcMotor.Direction.REVERSE);


        intake = hardwareMap.get(Servo.class, "intake");


        BillyArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BillyArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double raiseToPosition(int positionLevel, Double targetSpeed) {
        if (positionLevel == 0 && BillyArm.getCurrentPosition() < maxPosition)
        {
            BillyArm.setTargetPosition((int)(1));
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
            return positionLevel;
        }
        else if (positionLevel == 1 && BillyArm.getCurrentPosition() < maxPosition)
        {
            BillyArm.setTargetPosition((int)(armLevel1Position));
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
            return positionLevel;
        }
        else if (positionLevel == 2 && BillyArm.getCurrentPosition() < maxPosition)
        {
            BillyArm.setTargetPosition(armLevel2Position);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
        else if (positionLevel == 3 && BillyArm.getCurrentPosition() < maxPosition)
        {
            BillyArm.setTargetPosition(armLevel3Position);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
        else if (positionLevel == 4 && BillyArm.getCurrentPosition() < maxPosition)
        {
            BillyArm.setTargetPosition(armLevel4Position);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.4);
        }
        else {
            armPosition =(int)(0);
            BillyArm.setTargetPosition(armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
     /*
        if(positionLevel == '1' && BillyArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel1Position;
            getMotorPosition();
            BillyArm.setTargetPosition(-armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
        else if(positionLevel == '2' && BillyArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel2Position;
            BillyArm.setTargetPosition(armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);

        }
        else if(positionLevel == '3' && BillyArm.getCurrentPosition() < maxPosition){
            armPosition = armLevel3Position;
            BillyArm.setTargetPosition(armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);

        }
        else if (positionLevel == '0' && BillyArm.getCurrentPosition() > minPosition){
            armPosition = armStowPosition;
            BillyArm.setTargetPosition(armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
        else {
            armPosition =(int)(COUNTS_PER_DEGREE * 90);;
            BillyArm.setTargetPosition(armPosition);
            BillyArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BillyArm.setPower(0.5);
        }
*/

        return 0;
    }

    public int getMotorPosition() {
        return BillyArm.getCurrentPosition();
    }
}   //end program
