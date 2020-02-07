package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by Gabriel on 2018-06-12.
 * UNTESTED.
 */


@Autonomous (name = "Blue Foundation Wall", group = "Qual")
public class BlueFoundationWall extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    public Servo intakeServoLeft;
    public Servo intakeServoRight;

    public Servo foundationServoLeft;
    public Servo foundationServoRight;

    public Servo skystoneServo;
    public double skystoneServoPosition = StemperFiConstants.SKYSTONE_SERVO_OPEN;

    public Servo extensionServo;
    public double extensionServoPosition = StemperFiConstants.EXTENSION_SERVO_IN;

    public boolean isBlue = true;
    public boolean isBridge = false;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();  //Figure out why the naive one doesn't have a public constructor
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());

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

        while (!opModeIsActive() && !isStopRequested()) {
            composeTelemetry();
            telemetry.update();
        }
        telemetry.addData("Go Go Go!", "HI");
        telemetry.update();

        int length = (int) Math.floor(45.5 * StemperFiConstants.TICKS_PER_CM); // 45.5 cm
        int slide = isBlue ? 900 : -900;
        //32cm;
        forward(length, 0.4);
        slideLeft(slide, 0.4);
        length = (int) Math.floor(38 * StemperFiConstants.TICKS_PER_CM);
        forward(length, 0.4);
        sleep(500);
        turn(100, .3);
        turn(-100, .3);
        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_DOWN);
        sleep(1000);
        length = (int) Math.floor(-90 * StemperFiConstants.TICKS_PER_CM);
        forward(length, 0.4);

        turn(100, .5);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double power = 0.8;
        if (isBlue) {
            frontRight.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(opModeIsActive() && angles.firstAngle < 90) {
                composeTelemetry();
                telemetry.update();
            }
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
            }
        } else {
            frontRight.setPower(-power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            backLeft.setPower(power);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(opModeIsActive() && angles.firstAngle > -90) {
                composeTelemetry();
                telemetry.update();
            }
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
            }
        }

        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_UP);
        sleep(1000);
        length = (int) Math.floor(-30 * StemperFiConstants.TICKS_PER_CM);
        forward(length, 0.6);
        slide = isBlue ? 800 : -800;
        slideLeft(slide, 0.6);
        // park closer to the bridge
        if (isBridge) {
            int squareSlide = 66 * StemperFiConstants.SLIDE_TICKS_PER_CM;
            if (isBlue) {
                slideRight(squareSlide, .50);
            } else {
                slideLeft(squareSlide, .50);
            }
        }
        length = (int) Math.floor(-70 * StemperFiConstants.TICKS_PER_CM);
        forward(length, 0.6);
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
            telemetry.update();
        }
    }

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

    public void slideRight(int ticks, double power) {
        slideLeft(-ticks, power);
    }

    public void turn(int ticks, double power) {
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);

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

    void composeTelemetry() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}