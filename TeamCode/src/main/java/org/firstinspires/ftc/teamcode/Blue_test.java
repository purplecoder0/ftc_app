package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.proto_new.AutonomousMode;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Blue_test", group = "Autonomous")
//@Disabled
public class Blue_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();
    }

    protected void runOp() throws InterruptedException {

        grab_cube(true);
        wait(0.5);

        cubes_to_position(1, 1500);

        ball_blue();
        wait(0.5);

        cubes_to_position(-1, 0);

        grab_cube(false);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
