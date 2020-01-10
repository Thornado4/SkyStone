package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mando TeleOp", group="OpMode")
public class SteveTeleop extends OpMode {

    SteveBase steve;

    @Override
    public void init() {
        steve = new SteveBase(this);
    }

    @Override
    public void loop() {
        steve.postTelemetry();
        steve.resetHeading();
        steve.switchCollection();
        steve.updateDriveTrain();
        steve.controlCollection();
        steve.controlFoundationServos();
        steve.controlServoClaw();
        steve.parkServo();
        steve.controlServoGrabberSwivel(gamepad2.b, gamepad2.x);
        steve.controlLift(gamepad2.right_stick_y, gamepad2.left_stick_y);
    }
}