package org.firstinspires.ftc.teamcode.proto_new;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Purplecoder 27/01/2018.
 */

public abstract class AutonomousMode extends LinearOpMode {
    // Motors
    protected DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;

    // Servos
    protected Servo servoArm = null;
    protected Servo servoColor = null;
    protected Servo servoCubesLeft = null;
    protected Servo servoCubesRight = null;

    // Sensors
    protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;
    //protected OpticalDistanceSensor odsSensor = null;
    //protected ModernRoboticsI2cRangeSensor rangeSensor = null;

    // Constants
    protected static final double ARM_UP = 0.96;
    protected static final double ARM_DOWN = 0.33;
    protected static final double COLOR_FORWARD = 0.0;
    protected static final double COLOR_BACK = 1.0;
    protected static final double MID_SERVO = 0.5;
    protected static final double CUBES_MIN = 0.65;
    protected static final double CUBES_MAX = 0.8;
    protected static final double LIFT_MAX = 5000;
    protected static final double COUNTS_PER_CM = 67;

    protected ElapsedTime runtime = new ElapsedTime();

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        while (!isStopRequested() && gyroSensor.isCalibrating() && opModeIsActive()) {
            sleep(50);
            idle();
        }
        //telemetry.addData("Gyro: ", gyroSensor.getHeading());
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for Start button to be pressed
        waitForStart();
        try {
            runOp();
        } catch(Exception e) {
            e.printStackTrace();
        } finally {
            exitOpMode();
        }
    }

    protected abstract void initOpMode() throws InterruptedException;
    protected abstract void runOp() throws InterruptedException;
    protected abstract void exitOpMode() throws InterruptedException;

    protected void initHardware() {
        // Map the motors
        leftMotorF = hardwareMap.dcMotor.get("left_drive_front");
        leftMotorB = hardwareMap.dcMotor.get("left_drive_back");
        rightMotorF = hardwareMap.dcMotor.get("right_drive_front");
        rightMotorB = hardwareMap.dcMotor.get("right_drive_back");
        cubesMotor = hardwareMap.dcMotor.get("cubes");
        // Map the servos
        servoArm = hardwareMap.servo.get("arm");
        servoColor = hardwareMap.servo.get("color");
        servoCubesLeft = hardwareMap.servo.get("cubes_left");
        servoCubesRight = hardwareMap.servo.get("cubes_right");
        // Map the sensors
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cubesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the stopping method for wheels
        leftMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cubesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set the cubes mechanism direction
        cubesMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set servo directions
        servoArm.setDirection(Servo.Direction.FORWARD);
        servoColor.setDirection(Servo.Direction.FORWARD);
        servoCubesLeft.setDirection(Servo.Direction.FORWARD);
        servoCubesRight.setDirection(Servo.Direction.REVERSE);
        // Set the motors power to 0
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesRight.setPosition(CUBES_MAX);
        servoCubesLeft.setPosition(CUBES_MAX);

        // Calibrate sensors
        colorSensor.enableLed(true);
        gyroSensor.calibrate();

        //while(gyroSensor.getHeading() != 0);
    }

    // Wait for a number of seconds
    protected void wait(double seconds) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            //telemetry.addData("Time Elapsed: ", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }
    }
    //Wait

    // Autonomous ball selection
    protected void ball_auto(boolean red_team){

        servoColor.setPosition(MID_SERVO);
        wait(1.0);

        servoArm.setPosition(ARM_DOWN);
        wait(1.0);

        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        wait(1.0);

        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.update();

        if(red_team) {
            if(red > blue && red > green){
                servoColor.setPosition(COLOR_BACK);
            } else if(blue > red && blue > green){
                servoColor.setPosition(COLOR_FORWARD);
            }
        } else if(!red_team) {
            if (red > blue && red > green) {
                servoColor.setPosition(COLOR_FORWARD);
            } else if (blue > red && blue > green) {
                servoColor.setPosition(COLOR_BACK);
            }
        }

        wait(1.0);
        servoArm.setPosition(ARM_UP);
        wait(1.0);
        servoColor.setPosition(COLOR_BACK);
    }
    //AutonomousBall

    // Lift the cubes to the target specified with a desired power
    protected void cubes_to_position(int power, int target){
        target = (int)Range.clip(target, 0, LIFT_MAX);

        cubesMotor.setTargetPosition(target);
        cubesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cubesMotor.setPower(Range.clip(power, -1,1));

        while(cubesMotor.isBusy() && opModeIsActive()){
            idle();
        }

        cubesMotor.setPower(0);
        cubesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //CubesToPosition

    // Toggle function for grabbing the cube
    protected void grab_cube(boolean grab){
        if(grab){
            servoCubesLeft.setPosition(CUBES_MIN);
            servoCubesRight.setPosition(CUBES_MIN);
        } else {
            servoCubesLeft.setPosition(CUBES_MAX);
            servoCubesRight.setPosition(CUBES_MAX);
        }
    }
    //GrabCube

    // Gyro Rotation function using rampUp
    protected  void rotate_ticks(int target, int trigo){
        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);
        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);

        double power = 0.3, increment = 0.05, MAX_FWD = 0.9;
        boolean rampUp = true;


        while(opModeIsActive()&& (leftMotorB.isBusy() || leftMotorF.isBusy() ||
                rightMotorB.isBusy() || rightMotorF.isBusy())){

            int heading = gyroSensor.getHeading();
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.update();
            if(leftMotorF.getCurrentPosition() >= target || leftMotorB.getCurrentPosition() >= target ||
                    rightMotorF.getCurrentPosition() >= target ||rightMotorB.getCurrentPosition() >= target)
                break;
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += increment ;
                if (power >= MAX_FWD ) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= increment;
            }

            power = Range.clip(power, 0, 0.9);
            power_wheels(-trigo*power, -trigo*power, power, power);
            idle();
        }
        stopWheels();
    }
    //RotateTicks

    // Move forward to a target using encoders
    protected void run_forward(int target, int trigo) {
        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);
        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);

        double power = 0.3, increment = 0.02, MAX_FWD = 0.9;
        boolean rampUp = true;

        while(opModeIsActive()&& (leftMotorB.isBusy() || leftMotorF.isBusy() ||
                rightMotorB.isBusy() || rightMotorF.isBusy())){
            /*
            int heading = gyroSensor.getHeading();
            telemetry.addData("heading", "%3d deg", heading);
            telemetry.update();
            if(leftMotorF.getCurrentPosition() >= target || leftMotorB.getCurrentPosition() >= target ||
                    rightMotorF.getCurrentPosition() >= target ||rightMotorB.getCurrentPosition() >= target)
                break;
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += increment ;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                power -= increment;
            }
            */
            power = Range.clip(power, 0, 0.9);
            power_wheels(-trigo*power, -trigo*power, -trigo*power, -trigo*power);
            idle();
        }
        stopWheels();
    }
    //RunForward

    // Vuforia image recognition
    protected int activate_vuforia()  {
        int rez = -1, cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcO14uD/////AAAAmUILEtDJVkPbjdTr7NZGRE1UskK3HPIjKtoC7tONYCaYenUf9sRjducLbUEn0Cu8Eh3lrEVqM7VbF5MXcNfFyS39uTN3t7PtjK8HLcFpFsgTLRFwAGJlhcX+OFgqsjzPSiyE8v7Z+XIYwMhKf2Z2XmTQOCa6vXLL30nw3iLnE6J2Q5QFnNw/+AFLA881KCVYSeGBtujTRvfloxYCMYon30C1uwWB0txP4s7K1FukBiyfKScQFj7CwS+27BsSajo8lstaPwlSw5LssYO0cEbNQmi31q1meclqCkTL0nRVZcdj+UrfutQms0Ledjs6N8+bQg/qxo//KsFZ7pGZsCO3HIyrVKTEadLDzg+3KV7EJhNV";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        switch(vuMark) {
            case LEFT:
                rez = 1;
                break;

            case CENTER:
                rez = 2;
                break;

            case RIGHT:
                rez = 3;
                break;

            case UNKNOWN:
                rez = 0;
                break;
        }
        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.update();

        return rez;
    }
    //ActivateVuforia

    // Power the wheel motors with values given
    protected void power_wheels(double leftPowerF, double leftPowerB, double rightPowerF, double rightPowerB) {
        leftMotorF.setPower(Range.clip(leftPowerF, -1, 1));
        leftMotorB.setPower(Range.clip(leftPowerB, -1, 1));
        rightMotorF.setPower(Range.clip(rightPowerF, -1, 1));
        rightMotorB.setPower(Range.clip(rightPowerB, -1, 1));
    }
    //PowerWheels

    // Move the robot based on left and right powers
    protected void move(double leftWheelsPower, double rightWheelsPower) {
        leftMotorF.setPower(Range.clip(leftWheelsPower, -1, 1));
        leftMotorB.setPower(Range.clip(leftWheelsPower, -1, 1));
        rightMotorF.setPower(Range.clip(rightWheelsPower, -1, 1));
        rightMotorB.setPower(Range.clip(rightWheelsPower, -1, 1));
    }
    //Move

    // This function stops all the wheels of the robot
    protected void stopWheels() {
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
    }
    //StopWheels

    // This function stops all the motors of the robot
    protected void stopMotors() {
        leftMotorF.setPower(0);
        leftMotorB.setPower(0);
        rightMotorF.setPower(0);
        rightMotorB.setPower(0);
        cubesMotor.setPower(0);
    }
    //StopMotors

}