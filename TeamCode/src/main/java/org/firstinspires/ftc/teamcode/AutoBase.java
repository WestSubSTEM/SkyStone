package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import edu.spa.ftclib.internal.controller.ErrorTimeThresholdFinishingAlgorithm;
import edu.spa.ftclib.internal.controller.FinishableIntegratedController;
import edu.spa.ftclib.internal.controller.PIDController;
import edu.spa.ftclib.internal.drivetrain.HeadingableTankDrivetrain;
import edu.spa.ftclib.internal.drivetrain.MecanumDrivetrain;
import edu.spa.ftclib.internal.sensor.IntegratingGyroscopeSensor;
import edu.spa.ftclib.internal.state.Button;

/**
 * Created by Gabriel on 2018-06-12.
 * UNTESTED.
 */

@Disabled
@Autonomous (name = "Auto Base", group = "qual")

public abstract class AutoBase extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    public static final int TICKS_PER_CM = 12;

//    private Button buttonA, buttonB, buttonX, buttonY;

//    private DcMotor liftLeftMotor;
//    private DcMotor liftMiddleMotor;
//    private DcMotor liftRightMotor;
//    private DcMotor[] liftMotors;

    public Servo intakeServoLeft;
    public Servo intakeServoRight;

    public Servo foundationServoLeft;
    public Servo foundationServoRight;

    public Servo skystoneServo;
    public double skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;

    public Servo extensionServo;
    public double extensionServoPosition = StemperFiConstants.EXTENSION_SERVO_IN;

    public boolean isShort;
    public boolean isLeft;

    AutoBase(boolean isShort, boolean isLeft) {
        this.isShort = isShort;
        this.isLeft = isLeft;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
//        liftLeftMotor = hardwareMap.get(DcMotor.class, "liftLeftMotor");
//        liftMiddleMotor = hardwareMap.get(DcMotor.class, "liftMiddleMotor");
//        liftRightMotor = hardwareMap.get(DcMotor.class, "liftRightMotor");
//        liftMiddleMotor.setDirection(DcMotor.Direction.REVERSE);
//        liftMotors = new DcMotor[3];
//        liftMotors[0] = liftLeftMotor;
//        liftMotors[1] = liftMiddleMotor;
//        liftMotors[2] = liftRightMotor;
//        for (DcMotor motor : liftMotors) {
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeRight");

        foundationServoRight = hardwareMap.get(Servo.class, "foundationServoRight");
        foundationServoLeft = hardwareMap.get(Servo.class, "foundationServoLeft");

        skystoneServo = hardwareMap.get(Servo.class, "skystoneServo");
        skystoneServo.setPosition(skystoneServoPosition);

        extensionServo = hardwareMap.get(Servo.class, "extensionServo");
        extensionServo.setPosition(extensionServoPosition);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        telemetry.addData("Go Go Go!", "HI");
        //forward(-500, 0.3);
        //slideLeft(500, .8);
        //turn(550, .8);
        int length = isShort ? -100 : -600;
        int slide = isLeft ? 900 : - 900;
        //forward(length, 0.8);
        sleep(10*1000);
        forward(600, 0.5);
    }

    public void forward(int ticks, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
        }
        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
        }
    }

    public void slideLeft(int ticks, double power) {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(ticks);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(power);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(ticks);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(power);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(-ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(power);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setTargetPosition(-ticks);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(power);
        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }

    public void slideRight(int ticks, double power) {
        slideLeft(-ticks, power);
    }

    public void turn(int ticks, double power) {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(ticks);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(power);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setTargetPosition(ticks);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setPower(power);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(-ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(power);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setTargetPosition(-ticks);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setPower(power);
        while (backLeft.isBusy() && opModeIsActive()) {
            telemetry.addData("Target: ", ticks);
            telemetry.addData("Position: ", frontLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}