package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="The Robot Autonomous", group="LinearOpMode")
public class TherobotAutonomous extends LinearOpMode {

    public TherobotBase therobot;

    @Override
    public void runOpMode() throws InterruptedException {
        therobot = new TherobotBase(this);
        waitForStart();
        therobot.timerOpMode.reset();
        // Right now this is just a test class for mecanum driving so I can refine the mecanum drive ASAP.
        // Just replace it with your own code once this is on your laptop.
        therobot.encoderDriveMecanum(true, 1, 0, 6);
        therobot.encoderDriveMecanum(true, 1, 6, 6);
        therobot.encoderDriveMecanum(true, 1, 6, 0);
        therobot.encoderDriveMecanum(true, 0.7, 6, -6);
        therobot.encoderDriveMecanum(true, 0.7, 0, -6);
        therobot.encoderDriveMecanum(true, 1, -6, -6);
        therobot.encoderDriveMecanum(true, 1, -6, 0);
        therobot.encoderDriveMecanum(true, 1, -6, 6);
        therobot.imuTurn(90);
    }

    // See how short that was? :D
}
