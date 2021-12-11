package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utilities.DriveUtil;

@TeleOp(name="Lights Example", group="Linear Opmode")
public class LEDLightsTutorial extends OpMode {

    RevBlinkinLedDriver lights;
    DriveUtil drive = new DriveUtil();
    int temp = 1;

    public void init(){
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
    }

    public void loop(){
        if(temp == 1){
            resetStartTime();
            temp = 2;
        }
        if(time >= 90){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
    }
}
