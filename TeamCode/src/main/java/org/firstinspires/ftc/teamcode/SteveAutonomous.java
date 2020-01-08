package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Steve Autonomous", group="")
public class SteveAutonomous extends LinearOpMode {

    SteveBase steve;

    @Override
    public void runOpMode() throws InterruptedException {
        steve = new SteveBase(this);
        steve.selection();
        waitForStart();
        //if(steve.pushingFoundation) steve.scoreFoundation();
        if (steve.pushingFoundation) steve.turnFoundation();

    }
}