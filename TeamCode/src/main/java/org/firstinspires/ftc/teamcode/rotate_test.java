package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Purplecoder on 27/01/2018.
 */

@Autonomous(name = "Rotate_test", group = "Autonomous")
@Disabled
public class rotate_test extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Done!", "play");
        telemetry.update();
    }

    protected void runOp() throws InterruptedException {

        //gyro_turn_right(90);
        rotate_ticks(800, 1);

        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(1000);
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
