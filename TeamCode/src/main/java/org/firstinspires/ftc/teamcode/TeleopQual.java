package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import edu.spa.ftclib.internal.state.Button;

import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;

@TeleOp(name = "Qual Tele-op", group = "Qual")

public class TeleopQual extends OpMode {
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    // The three lift motors
    private DcMotor liftLeftMotor;
    private DcMotor liftMiddleMotor;
    private DcMotor liftRightMotor;
    private DcMotor[] liftMotors;

    // Intake servos VEX 2-Wire Motor 393
    public Servo intakeServoLeft;
    public Servo intakeServoRight;

    // Servos for grabbing the foundation
    public Servo foundationServoLeft;
    public Servo foundationServoRight;

    // servo for grabbing the bricks by the top nub
    public Servo skystoneServo;
    public double skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;

    // servo for extending brick to foundation
    public Servo extensionServo;
    public double extensionServoPosition = StemperFiConstants.EXTENSION_SERVO_IN;

    // The MecanumDrivetrain courtous of HOMAR FTC library
    public MecanumDrivetrain drivetrain;


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        // Setup teh drivetrain
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        drivetrain = new MecanumDrivetrain(driveMotors);

        // our amazing 3 motor lift
        liftLeftMotor = hardwareMap.get(DcMotor.class, "liftLeftMotor");
        liftMiddleMotor = hardwareMap.get(DcMotor.class, "liftMiddleMotor");
        liftRightMotor = hardwareMap.get(DcMotor.class, "liftRightMotor");
        liftMotors = new DcMotor[] {liftLeftMotor, liftMiddleMotor, liftRightMotor};
        // we have to reverse the middle motor because the chain goes around it in the opposite direction
        liftMiddleMotor.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor motor : liftMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // We do not need break because our lead screw won't turn due to gravity
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Initialize the Servos
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
        // Driver 1
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
        if (gamepad2.right_trigger > 0.1) {
            // Left servo intake .5 -> .9
            intakeServoPowerLeft = (gamepad2.right_trigger * StemperFiConstants.VEX_EDR_393_MAX_RANGE) + StemperFiConstants.VEX_EDR_393_OFF;
            // Right servo intake .5 -> .1
            intakeServoPowerRight = StemperFiConstants.VEX_EDR_393_OFF - (gamepad2.right_trigger * StemperFiConstants.VEX_EDR_393_MIN_RANGE);
        } else if (gamepad2.left_trigger > 0.1) {
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
        if (gamepad2.left_stick_x > 0.1 && !(gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            extensionServoPosition -= 0.01;
            extensionServoPosition = extensionServoPosition > StemperFiConstants.EXTENSION_SERVO_OUT ? extensionServoPosition : StemperFiConstants.EXTENSION_SERVO_OUT;
        } else if (gamepad2.left_stick_x < -0.1 && !(gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            extensionServoPosition += 0.01;
            extensionServoPosition = extensionServoPosition < StemperFiConstants.EXTENSION_SERVO_IN ? extensionServoPosition : StemperFiConstants.EXTENSION_SERVO_IN;
        }
        extensionServo.setPosition(extensionServoPosition);

        // lift
        double power = (gamepad2.right_stick_y * -1);
        if (Math.abs(power) > 0.1) {
            for (DcMotor motor : liftMotors) {
                motor.setPower(power);
            }
        }
        telemetry.update();
    }
}

