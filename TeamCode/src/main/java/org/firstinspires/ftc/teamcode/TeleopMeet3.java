package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import edu.spa.ftclib.internal.state.Button;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

/**
 * Created by Michaela on 1/3/2018.
 */


@TeleOp(name = "Meet 3 Tele-op", group = "meet3")

public class TeleopMeet3 extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    private Button buttonA, buttonB, buttonX, buttonY;

    private DcMotor liftLeftMotor;
    private DcMotor liftMiddleMotor;
    private DcMotor liftRightMotor;
    private DcMotor[] liftMotors;

    public Servo intakeServoLeft;
    public Servo intakeServoRight;

    public Servo foundationServoLeft;
    public Servo foundationServoRight;

    public Servo skystoneServo;
    public double skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;

    public Servo extensionServo;
    public double extensionServoPosition = StemperFiConstants.EXTENSION_SERVO_IN;

    public MecanumDrivetrain drivetrain;
    public boolean manual = false;
    public int liftLevel = 0;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        buttonA = new Button();
        buttonB = new Button();
        buttonX = new Button();
        buttonY = new Button();

        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");

        drivetrain = new MecanumDrivetrain(new DcMotor[]{frontLeft, frontRight, backLeft, backRight});

        liftLeftMotor = hardwareMap.get(DcMotor.class, "liftLeftMotor");
        liftMiddleMotor = hardwareMap.get(DcMotor.class, "liftMiddleMotor");
        liftRightMotor = hardwareMap.get(DcMotor.class, "liftRightMotor");
        liftMiddleMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotors = new DcMotor[3];
        liftMotors[0] = liftLeftMotor;
        liftMotors[1] = liftMiddleMotor;
        liftMotors[2] = liftRightMotor;
        for (DcMotor motor : liftMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeRight");

        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_UP);
        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");
        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_UP);

        skystoneServo = hardwareMap.get(Servo.class, "skystoneServo");
        skystoneServo.setPosition(skystoneServoPosition);

        extensionServo = hardwareMap.get(Servo.class, "extensionServo");
        extensionServo.setPosition(extensionServoPosition);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rotation = -gamepad1.left_stick_x;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);

        // Driver 2
        if (gamepad2.left_stick_button || gamepad2.right_stick_button) {
            foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_DOWN);
            foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_DOWN);
        } else {
            foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_UP);
            foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_UP);
        }

        // Intake motors
        double intakeServoPowerLeft = 0.5;
        double intakeServoPowerRight = 0.5;
        if (gamepad2.right_trigger > 0.2) {
            // Left servo intake .5 -> .9
            intakeServoPowerLeft = (gamepad2.right_trigger * StemperFiConstants.VEX_EDR_393_MAX_RANGE) + StemperFiConstants.VEX_EDR_393_OFF;
            // Right servo intake .5 -> .1
            intakeServoPowerRight = StemperFiConstants.VEX_EDR_393_OFF - (gamepad2.right_trigger * StemperFiConstants.VEX_EDR_393_MIN_RANGE);
        } else if (gamepad2.left_trigger > 0.2) {
            // Left servo outTake .5 -> .1
            intakeServoPowerLeft = StemperFiConstants.VEX_EDR_393_OFF - (gamepad2.left_trigger * StemperFiConstants.VEX_EDR_393_MIN_RANGE);
            // Right servo outTake .5 -> .9
            intakeServoPowerRight = (gamepad2.left_trigger * StemperFiConstants.VEX_EDR_393_MAX_RANGE) + StemperFiConstants.VEX_EDR_393_OFF;
        }
        intakeServoLeft.setPosition(intakeServoPowerLeft);
        intakeServoRight.setPosition(intakeServoPowerRight);

        // Skystone Servo
        if (gamepad2.left_bumper) {
            skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_CLOSE;
        } else if (gamepad2.right_bumper) {
            skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;
        }
        skystoneServo.setPosition(skystoneServoPosition);

        // Extension Servo
        if (gamepad2.left_stick_x > 0.2) {
            extensionServoPosition -= 0.01;
            extensionServoPosition = extensionServoPosition > StemperFiConstants.EXTENSION_SERVO_OUT ? extensionServoPosition : StemperFiConstants.EXTENSION_SERVO_OUT;
        } else if (gamepad2.left_stick_x < -0.2) {
            extensionServoPosition += 0.01;
            extensionServoPosition = extensionServoPosition < StemperFiConstants.EXTENSION_SERVO_IN ? extensionServoPosition : StemperFiConstants.EXTENSION_SERVO_IN;
        }
        extensionServo.setPosition(extensionServoPosition);

        // lift

        double power = (gamepad2.right_stick_y * -1);
        if (manual || Math.abs(power) > 0.2) {
            // manual mode
            for (DcMotor motor : liftMotors) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(power);
            }
            manual = power != 0;
            liftLevel = 0;
        } else {
            buttonX.input(gamepad2.x);
            buttonY.input(gamepad2.y);
            buttonA.input(gamepad2.a);
            buttonB.input(gamepad2.b);
            if (buttonY.onPress()) {
                liftLevel++;
                liftLevel = liftLevel < StemperFiConstants.LIFT_POSITIONS.length ? liftLevel : StemperFiConstants.LIFT_POSITIONS.length - 1;
            }
            if (buttonA.onPress()) {
                liftLevel--;
                liftLevel = liftLevel < 0 ? liftLevel = 0 : liftLevel;
            }
            if (buttonB.onPress()) {
                int targetPosition = StemperFiConstants.LIFT_POSITIONS[liftLevel];
                for (DcMotor motor : liftMotors) {
                    motor.setTargetPosition(targetPosition);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(1.0);
                }
            }
            if (buttonX.onPress()) {
                liftLevel = 0;
                manual = false;
                for (DcMotor motor : liftMotors) {
                    motor.setTargetPosition(0);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.setPower(1.0);
                }
            }
        }
        telemetry.addData("Lift Level: ", liftLevel);
        telemetry.addData("r: ", liftRightMotor.getCurrentPosition());
        telemetry.addData("ext: ", extensionServoPosition);
        telemetry.addData( "button: ", gamepad2.left_stick_button);
        telemetry.update();
    }
}

