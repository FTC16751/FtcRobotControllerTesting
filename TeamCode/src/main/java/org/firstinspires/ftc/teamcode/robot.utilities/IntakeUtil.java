/* P3 2021-22 season Auto Control Program
 Hardware Names:
 Webcam
 Front_Left
 Front_Right
 Rear_Left
 Rear_Right

 */

package org.firstinspires.ftc.teamcode.robot.utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class IntakeUtil {

    Servo intake;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;

    /* Constructor */
    public IntakeUtil() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        intake = hardwareMap.get(Servo.class, "intake");
    }

    public void setIntake(int mode) {

        switch (mode)
    {
        case 1:
            intake.setPosition(1);
            break;
        case 2:
            intake.setPosition(-1);
            break;
        case 0:
            intake.setPosition(.5);
            break;
    }
}
}   //end program
