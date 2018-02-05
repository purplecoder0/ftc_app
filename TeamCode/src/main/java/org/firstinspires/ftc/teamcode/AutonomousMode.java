package org.firstinspires.ftc.teamcode.proto_new;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Purplecoder 27/01/2018.
 */

public abstract class AutonomousMode extends LinearOpMode {

    // Motors
    private DcMotor cubesMotor = null;
    protected DcMotor leftMotorF = null;
    protected DcMotor leftMotorB = null;
    protected DcMotor rightMotorF = null;
    protected DcMotor rightMotorB = null;
    // Servos
    private Servo servoArm = null;
    private Servo servoColor = null;
    private Servo servoCubesLeft = null;
    private Servo servoCubesRight = null;

    // Sensors

    protected ModernRoboticsI2cGyro gyroSensor = null;
    protected ColorSensor colorSensor = null;
    //protected OpticalDistanceSensor odsSensor = null;
    //protected ModernRoboticsI2cRangeSensor rangeSensor = null;


    // Constants
    private static final double ARM_UP = 0.96;
    private static final double ARM_DOWN = 0.25;
    private static final double COLOR_LEFT = 0.0;
    private static final double COLOR_RIGHT = 1.0;
    private static final double MID_SERVO = 0.5;
    private static final double CUBES_MIN = 0.65;
    private static final double CUBES_MAX = 0.8;
    private static final double LIFT_MAX = 5000;  //need to be set

    protected static final double COUNTS_PER_CM = 67;  //needs to be set

    // Additional helper variables
    private double leftWheelsPower = 0, rightWheelsPower = 0;
    private double deadzone = 0.1;
    private double cubesPower = 1;

    private boolean switchServoCubes = false;

    //Vuforia

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    protected ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        while (!isStopRequested() && gyroSensor.isCalibrating() && opModeIsActive()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Gyro: ", gyroSensor.getHeading());
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

    public void initHardware() {
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
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        // Set the wheel motors
        leftMotorF.setDirection(DcMotor.Direction.REVERSE);
        leftMotorB.setDirection(DcMotor.Direction.REVERSE);
        rightMotorF.setDirection(DcMotor.Direction.FORWARD);
        rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        // Initialize servo positions
        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(MID_SERVO);
        servoCubesRight.setPosition(CUBES_MAX);
        servoCubesLeft.setPosition(CUBES_MAX);
        
        // Calibrate sensors
        colorSensor.enableLed(true);
        gyroSensor.calibrate();

        while(gyroSensor.getHeading() != 0);

        wait(1.0);
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

    // Drive with encoders
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int     newLeftFTarget;
        int     newRightFTarget;
        boolean stop = false;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFTarget = leftMotorF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_CM);
            newRightFTarget = rightMotorF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_CM);

            leftMotorF.setTargetPosition(newLeftFTarget);
            rightMotorF.setTargetPosition(newRightFTarget);

            // Turn On RUN_TO_POSITION
            leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            move(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && !stop &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotorF.isBusy() && rightMotorF.isBusy())) {

                if(Math.abs(leftMotorF.getCurrentPosition()) > Math.abs(newLeftFTarget) ||
                        Math.abs(rightMotorF.getCurrentPosition()) > Math.abs(newRightFTarget)) {
                    stop = true;
                }

                // Display it for the driver.
                telemetry.addData("TargetF",  "%7d:%7d",      newLeftFTarget,  newRightFTarget);
                telemetry.addData("ActualF",  "%7d:%7d",      leftMotorF.getCurrentPosition(),
                        rightMotorF.getCurrentPosition());
                telemetry.update();
                idle();
            }

            // Stop all motion;
            stopWheels();

            // Turn off RUN_TO_POSITION
            leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
    //EncoderDrive

    // Turn using encoder, trigo = 1 for CW and trigo = -1 for CCW
    public void encoderTurn(double power, int distance){

        ///distance = 1680 for 90 degrees

        boolean stop = false;

        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorF.setTargetPosition(-distance);
        rightMotorB.setTargetPosition(distance);

        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        power_wheels(power,power,-power,-power);

        while(opModeIsActive() && !stop && (leftMotorF.isBusy() && leftMotorB.isBusy())){
            //wait until target position is reached

            if(Math.abs(leftMotorF.getCurrentPosition()) > Math.abs(distance) ||
                    Math.abs(rightMotorF.getCurrentPosition()) > Math.abs(distance)) {
                stop = true;
            }

            idle();
        }

        // Stop the wheels
        stopWheels();

        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //EncoderTurn

    // Power the wheel motors with values given
    protected void power_wheels(double leftPowerF, double leftPowerB, double rightPowerF, double rightPowerB) {
        leftMotorF.setPower(Range.clip(leftPowerF, -1, 1));
        leftMotorB.setPower(Range.clip(leftPowerB, -1, 1));
        rightMotorF.setPower(Range.clip(rightPowerF, -1, 1));
        rightMotorB.setPower(Range.clip(rightPowerB, -1, 1));
    }

    // Move the robot based on left and right powers
    protected void move(double leftWheelsPower, double rightWheelsPower) {
        leftMotorF.setPower(Range.clip(leftWheelsPower, -1, 1));
        leftMotorB.setPower(Range.clip(leftWheelsPower, -1, 1));
        rightMotorF.setPower(Range.clip(rightWheelsPower, -1, 1));
        rightMotorB.setPower(Range.clip(rightWheelsPower, -1, 1));
    }
    //Move

    // Move the robot forward or backward
    protected void forward(double power) {
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorF.setPower(power);
        rightMotorB.setPower(power);
    }
    //Forward

    // Turn the robot CW for + vals and CCW for - vals
    protected void turn(double power) {
        leftMotorF.setPower(power);
        leftMotorB.setPower(power);
        rightMotorF.setPower(-power);
        rightMotorB.setPower(-power);
    }
    //Turn

    protected void move_distance(double power, double distance, int trigo) {

        boolean stop = false;

        if(trigo == 1) {
            leftMotorF.setDirection(DcMotor.Direction.FORWARD);
            leftMotorB.setDirection(DcMotor.Direction.FORWARD);
            rightMotorF.setDirection(DcMotor.Direction.REVERSE);
            rightMotorB.setDirection(DcMotor.Direction.REVERSE);
        }

        if(trigo == -1) {
            leftMotorF.setDirection(DcMotor.Direction.REVERSE);
            leftMotorB.setDirection(DcMotor.Direction.REVERSE);
            rightMotorF.setDirection(DcMotor.Direction.FORWARD);
            rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        }

        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftFTarget = leftMotorF.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
        int newRightFTarget = rightMotorF.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
        int newLeftBTarget = leftMotorF.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);
        int newRightBTarget = rightMotorF.getCurrentPosition() + (int)(distance * COUNTS_PER_CM);

        leftMotorF.setTargetPosition(newLeftFTarget);
        leftMotorB.setTargetPosition(newLeftBTarget);
        rightMotorF.setTargetPosition(newRightFTarget);
        rightMotorB.setTargetPosition(newRightBTarget);

        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double speed = Range.clip(Math.abs(power), -1.0, 1.0);

        power_wheels(speed, speed, speed, speed);

        while(opModeIsActive() && leftMotorF.isBusy() && leftMotorB.isBusy() && rightMotorF.isBusy() && rightMotorB.isBusy() && !stop) {

            telemetry.addData("CurrDistFront ", "%7d : %7d", leftMotorF.getCurrentPosition(), rightMotorF.getCurrentPosition());
            telemetry.addData("CurrDistBack ", "%7d : %7d", leftMotorB.getCurrentPosition(), rightMotorB.getCurrentPosition());
            telemetry.update();

            if(Math.abs(leftMotorF.getCurrentPosition()) > Math.abs(newLeftFTarget) ||
                    Math.abs(rightMotorF.getCurrentPosition()) > Math.abs(newRightFTarget) ||
                    Math.abs(leftMotorB.getCurrentPosition()) > Math.abs(newLeftBTarget) ||
                    Math.abs(rightMotorB.getCurrentPosition()) > Math.abs(newRightBTarget)) {
                stop = true;
            }
            idle();
        }


        stopWheels();
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Move_Distance


    protected void rotate_distance(double power, double distance, int trigo) {

        boolean stop = false;

        if(trigo == 1) {
            leftMotorF.setDirection(DcMotor.Direction.FORWARD);
            leftMotorB.setDirection(DcMotor.Direction.FORWARD);
            rightMotorF.setDirection(DcMotor.Direction.REVERSE);
            rightMotorB.setDirection(DcMotor.Direction.REVERSE);
        }

        if(trigo == -1) {
            leftMotorF.setDirection(DcMotor.Direction.REVERSE);
            leftMotorB.setDirection(DcMotor.Direction.REVERSE);
            rightMotorF.setDirection(DcMotor.Direction.FORWARD);
            rightMotorB.setDirection(DcMotor.Direction.FORWARD);
        }

        leftMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftFTarget = leftMotorF.getCurrentPosition() + (int) distance;
        int newRightFTarget = rightMotorF.getCurrentPosition() + (int) distance;
        int newLeftBTarget = leftMotorF.getCurrentPosition() + (int) distance;
        int newRightBTarget = rightMotorF.getCurrentPosition() + (int) distance;

        leftMotorF.setTargetPosition(-newLeftFTarget);
        leftMotorB.setTargetPosition(-newLeftBTarget);
        rightMotorF.setTargetPosition(newRightFTarget);
        rightMotorB.setTargetPosition(newRightBTarget);

        leftMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double speed = Range.clip(Math.abs(power), -1.0, 1.0);

        power_wheels(speed, speed, -speed, -speed);

        while(opModeIsActive() && leftMotorF.isBusy() && leftMotorB.isBusy() && rightMotorF.isBusy() && rightMotorB.isBusy() && !stop) {

            telemetry.addData("CurrDistFront ", "%7d : %7d", leftMotorF.getCurrentPosition(), rightMotorF.getCurrentPosition());
            telemetry.addData("CurrDistBack ", "%7d : %7d", leftMotorB.getCurrentPosition(), rightMotorB.getCurrentPosition());
            telemetry.update();

            if(Math.abs(leftMotorF.getCurrentPosition()) > Math.abs(newLeftFTarget) ||
                    Math.abs(rightMotorF.getCurrentPosition()) > Math.abs(newRightFTarget) ||
                    Math.abs(leftMotorB.getCurrentPosition()) > Math.abs(newLeftBTarget) ||
                    Math.abs(rightMotorB.getCurrentPosition()) > Math.abs(newRightBTarget)) {
                stop = true;
            }

        }

        stopWheels();
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Rotate_Distance

    protected void gyro_rotate(double power, int angle, int trigo) {

        boolean stop = false;

        if(trigo == 1) { // DREAPTA
            leftMotorF.setDirection(DcMotor.Direction.FORWARD);
            leftMotorB.setDirection(DcMotor.Direction.FORWARD);
            rightMotorF.setDirection(DcMotor.Direction.REVERSE);
            rightMotorB.setDirection(DcMotor.Direction.REVERSE);

            double speed = Range.clip(Math.abs(power), -1.0, 1.0);

            int curr = gyroSensor.getHeading();

            int target = curr + angle;

            if(target > 359) {  // Rotatia depaseste heading-ul de 360
                target = target - 360;

                while(opModeIsActive() && (curr > target)) {
                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                curr = gyroSensor.getHeading();

                if(Math.abs(target - curr) > 10)
                    speed = 0.3;

                while(opModeIsActive() && (curr < target)) {
                    if(target - curr <= 5 && speed > 0.1)
                        speed -= 0.05;

                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                while(opModeIsActive() && (curr > target)) {
                    power_wheels(0.05, 0.05, -0.05, -0.05);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();
            }

            else {
                power_wheels(-speed, -speed, speed, speed);
                wait(0.2);
                while(opModeIsActive() && (curr < target)) {
                    if(target - curr <= 10 && speed > 0.1)
                        speed -= 0.05;

                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                curr = gyroSensor.getHeading();

                while(opModeIsActive() && (curr > target)) {
                    power_wheels(0.05, 0.05, -0.05, -0.05);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();
            }
        }

        if(trigo == -1) {  // STANGA
            leftMotorF.setDirection(DcMotor.Direction.REVERSE);
            leftMotorB.setDirection(DcMotor.Direction.REVERSE);
            rightMotorF.setDirection(DcMotor.Direction.FORWARD);
            rightMotorB.setDirection(DcMotor.Direction.FORWARD);

            double speed = Range.clip(Math.abs(power), -1.0, 1.0);

            int curr = gyroSensor.getHeading();

            int target = curr - angle;

            if(target < 0) {
                target = 360 + target;

                while(opModeIsActive() && (curr < target)) {
                    if(curr <= 10 && speed >0.1)
                        speed -= 0.05;

                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                curr = gyroSensor.getHeading();

                if(Math.abs(target - curr) > 10)
                    speed = 0.3;

                while(opModeIsActive() && (curr > target)) {
                    if(target - curr <= 5 && speed > 0.1)
                        speed -= 0.05;

                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                while(opModeIsActive() && (curr < target)) {
                    power_wheels(0.05, 0.05, -0.05, -0.05);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();
            }

            else {
                power_wheels(-speed, -speed, speed, speed);
                wait(0.2);

                while(opModeIsActive() && (curr > target)) {
                    if(curr - target <= 10 && speed > 0.1)
                        speed -= 0.05;

                    power_wheels(-speed, -speed, speed, speed);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();

                curr = gyroSensor.getHeading();

                while(opModeIsActive() && curr < target) {
                    power_wheels(0.05, 0.05, -0.05, -0.05);
                    curr = gyroSensor.getHeading();
                }

                stopWheels();
            }
        }
    }

    protected void rotate_gyro(double power, int angle, int trigo) {

        if(trigo == 1) { // DREAPTA
            leftMotorF.setDirection(DcMotor.Direction.FORWARD);
            leftMotorB.setDirection(DcMotor.Direction.FORWARD);
            rightMotorF.setDirection(DcMotor.Direction.REVERSE);
            rightMotorB.setDirection(DcMotor.Direction.REVERSE);

            double speed = Range.clip(Math.abs(power), -1.0, 1.0);

            int curr = gyroSensor.getHeading();

            power_wheels(-speed, -speed, speed, speed);
            wait(1.0);

            while(opModeIsActive() && curr < angle) {
                power_wheels(-speed, -speed, speed, speed);
                curr = gyroSensor.getHeading();
            }

            stopWheels();

            curr = gyroSensor.getHeading();

            while(opModeIsActive() && curr > angle) {
                power_wheels(0.06, 0.06, -0.06, -0.06);
                curr = gyroSensor.getHeading();
            }

            stopWheels();

        }

        if(trigo == -1) {  // STANGA
            leftMotorF.setDirection(DcMotor.Direction.REVERSE);
            leftMotorB.setDirection(DcMotor.Direction.REVERSE);
            rightMotorF.setDirection(DcMotor.Direction.FORWARD);
            rightMotorB.setDirection(DcMotor.Direction.FORWARD);

            double speed = Range.clip(Math.abs(power), -1.0, 1.0);

            int curr = gyroSensor.getHeading();

            power_wheels(-speed, -speed, speed, speed);
            wait(1.0);

            while(opModeIsActive() && curr > angle) {
                power_wheels(-speed, -speed, speed, speed);
                curr = gyroSensor.getHeading();
            }

            stopWheels();

            curr = gyroSensor.getHeading();


            while(opModeIsActive() && curr < angle) {
                power_wheels(0.06, 0.06, -0.06, -0.06);
                curr = gyroSensor.getHeading();
            }

            stopWheels();

        }
    }

    protected void gyro_turn_pid(int angle){
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed ;
        double pGain = 0.003;  //need to be set
        double iGain = 0.002;  //need to be set
        double dGain = 0.01;  //need to be set
        int errorSum = 0;
        double dev = 10; //need to be set


        gyroSensor.calibrate();
        while(gyroSensor.getHeading() != 0);

        int curr = gyroSensor.getHeading();
        int error = 2;
        while(opModeIsActive() && error > 1) {
            curr = gyroSensor.getHeading();
            error = angle - curr;
            errorSum += error;

            speed = error * pGain + errorSum * iGain;
            speed = Range.clip(Math.abs(speed), -1.0, 1.0);
            power_wheels(-speed, -speed, -speed, -speed);
        }
    }

    protected  void rotate_ticks(int target, int trigo){
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);
        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);

        double power = 0.1, increment = 0.1, MAX_FWD = 0.9;
        boolean rampUp = true;


        while(opModeIsActive()&& (leftMotorB.isBusy() || leftMotorF.isBusy() ||
                rightMotorB.isBusy() || rightMotorF.isBusy())){
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
        }
        stopWheels();
    }

    protected void run_forward(int target) {
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotorF.setTargetPosition(target);
        rightMotorB.setTargetPosition(target);
        leftMotorF.setTargetPosition(target);
        leftMotorB.setTargetPosition(target);

        double power = 0.1, increment = 0.1, MAX_FWD = 0.9;
        boolean rampUp = true;


        while(opModeIsActive()&& (leftMotorB.isBusy() || leftMotorF.isBusy() ||
                rightMotorB.isBusy() || rightMotorF.isBusy())){
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
            power_wheels(power, power, power, power);
        }
        stopWheels();
    }

    protected void run_forward_pid(int power){
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed, speedL, speedR;
        double pGain = 0.3;  //need to be set
        double iGain = 0.2;  //need to be set
        int errorSum = 0;
        double dGain = 0.1;  //need to be set
        double dev;

        speed = speedL = speedR = Range.clip(Math.abs(power), -1.0, 1.0);

        gyroSensor.calibrate();
        while(gyroSensor.getHeading() != 0);

        int curr = gyroSensor.getHeading();
        int angle = curr;
        int error = 2;

        while(opModeIsActive() && error > 1) {
            curr = gyroSensor.getHeading();
            error = angle - curr;
            errorSum += error;

            if(error > 0){
                speedL = speed + error * pGain + errorSum * iGain;
                speedR = speed - error * pGain - errorSum * iGain;
            } else {
                speedL = speed - error * pGain - errorSum * iGain;
                speedR = speed + error * pGain + errorSum * iGain;
            }

            speedL = Range.clip(Math.abs(speedL), -1.0, 1.0);
            speedR = Range.clip(Math.abs(speedR), -1.0, 1.0);
            power_wheels(speedL, speedL, speedR, speedR);
        }
    }

    protected void ball_red(){

        servoColor.setPosition(MID_SERVO);
        wait(2.0);

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

        if(red > blue && red > green){
            servoColor.setPosition(COLOR_LEFT);
        } else if(blue > red && blue > green){
            servoColor.setPosition(COLOR_RIGHT);
        }

        wait(3.0);

        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_RIGHT);

    }

    protected void ball_blue(){

        servoColor.setPosition(MID_SERVO);
        wait(2.0);

        servoArm.setPosition(ARM_DOWN);
        wait(1.0);


        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.update();

        wait(1.0);

        if(blue > red && blue > green){
            servoColor.setPosition(COLOR_RIGHT);
        } else if(red > blue && red > green){
            servoColor.setPosition(COLOR_LEFT);
        }

        wait(3.0);

        servoArm.setPosition(ARM_UP);
        servoColor.setPosition(COLOR_RIGHT);

    }

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

    protected void grab_cube(boolean ok){
        if(ok == true){
            servoCubesLeft.setPosition(CUBES_MIN);
            servoCubesRight.setPosition(CUBES_MIN);
        } else{
            servoCubesLeft.setPosition(CUBES_MAX);
            servoCubesRight.setPosition(CUBES_MAX);
        }

    }

    protected int Vuforia()  {
        int rez = 0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcO14uD/////AAAAmUILEtDJVkPbjdTr7NZGRE1UskK3HPIjKtoC7tONYCaYenUf9sRjducLbUEn0Cu8Eh3lrEVqM7VbF5MXcNfFyS39uTN3t7PtjK8HLcFpFsgTLRFwAGJlhcX+OFgqsjzPSiyE8v7Z+XIYwMhKf2Z2XmTQOCa6vXLL30nw3iLnE6J2Q5QFnNw/+AFLA881KCVYSeGBtujTRvfloxYCMYon30C1uwWB0txP4s7K1FukBiyfKScQFj7CwS+27BsSajo8lstaPwlSw5LssYO0cEbNQmi31q1meclqCkTL0nRVZcdj+UrfutQms0Ledjs6N8+bQg/qxo//KsFZ7pGZsCO3HIyrVKTEadLDzg+3KV7EJhNV";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

            if(true){
                rez = 1;
            }
            if(true){
                rez = 2;
            }
            else
                rez = 3;

            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.update();
        }
        return rez;
    }

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
