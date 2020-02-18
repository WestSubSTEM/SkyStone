package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "Auto Park", group = "Qual")
public class AutoPark extends LinearOpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    // Intake servos VEX 2-Wire Motor 393
    public Servo intakeServoLeft;
    public Servo intakeServoRight;

    // Servos for grabbing the foundation
    public Servo foundationServoLeft;
    public Servo foundationServoRight;

    // servo for grabbing the bricks by the top nub
    public Servo skystoneServo;
    public double skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;


    // servo for extending brick to foundtion
    public Servo extensionServo;
    public double extensionServoPosition = StemperFiConstants.EXTENSION_SERVO_IN;

    // How many seconds to sleep before moving (let teammate move first)
    public int sleepSec = 0;

    // Move moveForward or moveBackwards (always keep intake facing 6 blocks)
    public boolean isForward = true;

    // Do we move sideways to bridge?
    public boolean isSideways = false;
    // If so do we move right or left
    public boolean isLeft = false;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        // Put all the drive motors in an array when we want to do the same thing for all the motors
        // we can loop through them
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        // reverse the left side so that motors turn in the same relative direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize the Servos Even though we aren't using them set them to teleop initial state
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeRight");

        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");
        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");

        skystoneServo = hardwareMap.get(Servo.class, "skystoneServo");
        skystoneServo.setPosition(skystoneServoPosition);

        extensionServo = hardwareMap.get(Servo.class, "extensionServo");
        extensionServo.setPosition(extensionServoPosition);

        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_UP);

        // When was the last time a sleep button was pressed
        long lastSleepButtonPressed = 0;
        // Right now
        long now = 0;
        while (!opModeIsActive() && !isStopRequested()) {
            // because loop goes very quickly we have to put limits on how quickly
            // we will increment counter
            if (gamepad2.y) {
                now = System.currentTimeMillis();
                if (now - lastSleepButtonPressed > 1000) {
                    lastSleepButtonPressed = now;
                    // Limit sleep to 25 seconds so we don't over sleep autonomous
                    sleepSec = sleepSec == 25 ? 25 : sleepSec + 1;
                }
            } else if(gamepad2.a) {
                now = System.currentTimeMillis();
                if (now - lastSleepButtonPressed > 1000) {
                    lastSleepButtonPressed = now;
                    sleepSec = sleepSec == 0 ? 0 : sleepSec - 1;
                }
            } else {
                // a sleep button was not pressed so clear out the delay
                lastSleepButtonPressed = 0;
            }
            // Do we go moveForward or moveBackwards to the tape line
            if (gamepad2.x) {
                isForward = false;
            }
            else if (gamepad2.b) {
                isForward = true;
            }
            // Slide to the right or left to get close to the bridge
            if (gamepad2.left_bumper) {
                isSideways = true;
                isLeft = true;
            } else if (gamepad2.right_bumper) {
                isSideways = true;
                isLeft = false;
            } else if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
                // need a way to clear things out use those thumb stick buttons
                isSideways = false;
            }
            // show the status
            telemetry.addData("Move (x=Backwards, b=Forwards)", isForward ? "Forward" : "Reverse");
            telemetry.addData("Sleep Seconds (y=+1, a=-1", sleepSec);
            telemetry.addData("Sideways (rBumper=right, lBumper=left, thumbButton=clear)", isSideways);
            if (isSideways) {
                telemetry.addData("Sideways", isLeft ? "Left" : "Right");
            } else {
                telemetry.addData("Sideways", "None going straight");
            }
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        telemetry.addData("Go Robot", "Go!");
        telemetry.update();

        // first sleep while teammate is doing their thing
        sleep(1000 * sleepSec);

        // Do we go towards the bridge
        if (isSideways) {
            int squareSlide = 66 * StemperFiConstants.SLIDE_TICKS_PER_CM;
            if (isLeft) {
                slideLeft(squareSlide, 0.5);
            } else {
                slideRight(squareSlide, 0.5);
            }
        }

        // go forwards or moveBackwards
        int moveToLineLength = 50 * StemperFiConstants.TICKS_PER_CM;
        if (isForward) {
            moveForward(moveToLineLength, 0.5);
        } else {
            moveBackwards(moveToLineLength, .5);
        }
    }

    // Move the robot forwards
    public void moveForward(int ticks, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    // Move the robot backwards
    public void moveBackwards(int ticks, double power) {
        // backwards is just negative forwards
        moveForward(-ticks, power);
    }

    // strafe robot to the left
    public void slideLeft(int ticks, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }

        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    // strafe robot to the right
    public void slideRight(int ticks, double power) {
        // right is just negative left
        slideLeft(-ticks, power);
    }

}