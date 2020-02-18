package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous (name = "Auto Foundation", group = "Qual")
public class AutoFoundation extends LinearOpMode {

    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor[] driveMotors;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

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

    // Are we blue alliance.
    public boolean isBlue = true;

    // Park on the bridge side or the wall side
    public boolean isBridge = false;


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

        // get and initialize the IMU taken from FTC samples
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled = true;   //For debugging
        parameters.loggingTag = "IMU";      //For debugging
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());

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

        // Wait for the driving coach to press start
        while (!opModeIsActive() && !isStopRequested()) {
            // continuously print some telemetry to keep phone from crashing
            composeTelemetry();

            // Do we go moveForward or moveBackwards to the tape line
            if (gamepad2.x) {
                isBlue = true;
            }
            else if (gamepad2.b) {
                isBlue = false;
            }

            // Do we park on the wall or near the brdige
            if (gamepad2.y) {
                isBridge = true;
            } else if (gamepad2.a) {
                isBridge = false;
            }
            telemetry.addData("Alliance (x=Blue, b=Red)", isBlue ? "Blue" : "Red");
            telemetry.addData("Park (y=Bridge, a=Wall", isBridge ? "Bridge" : "Wall");
            telemetry.update();
        }
        telemetry.addData("Go Robot", "Go!");
        telemetry.update();

        // Move to foundation
        int length = (int) Math.floor(45.5 * StemperFiConstants.TICKS_PER_CM); // 45.5 cm
        moveForward(length, 0.4);
        // Get closer to middle of foundation
        int slideToCenterFoundation = 36 * StemperFiConstants.SLIDE_TICKS_PER_CM; //36 cm
        if (isBlue) {
            slideLeft(slideToCenterFoundation, 0.4);
        } else {
            slideRight(slideToCenterFoundation, 0.4);
        }
        // get all the way to foundaiton
        length = (int) Math.floor(38 * StemperFiConstants.TICKS_PER_CM);
        moveForward(length, 0.4);
        // let compliant intake wheels expand
        sleep(500);
        // nudge to left & right to account for intake wheels bouncing foundation forward
        turn(100, .3);
        turn(-100, .3);
        // Drop the servos hooks!!!!
        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_DOWN);
        // give the servos time to travel
        sleep(1000);
        // back up to wall
        length = (int) Math.floor(90 * StemperFiConstants.TICKS_PER_CM);
        moveBackwards(length, 0.4);


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
        // release the foundation hooks
        foundationServoLeft.setPosition(StemperFiConstants.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(StemperFiConstants.FOUNDATION_SERVO_RIGHT_UP);
        // wait for hooks to finish moving
        sleep(1000);

        // move backwards towards the line
        length = (int) Math.floor(30 * StemperFiConstants.TICKS_PER_CM);
        moveBackwards(length, 0.6);

        // move against the wall to straighten us out
        int slideToWall = 32 * StemperFiConstants.SLIDE_TICKS_PER_CM;
        if (isBlue) {
            slideLeft(slideToWall, 0.6);
        } else {
            slideRight(slideToWall, 0.6);
        }
        // park closer to the bridge?
        if (isBridge) {
            int squareSlide = 66 * StemperFiConstants.SLIDE_TICKS_PER_CM;
            if (isBlue) {
                slideRight(squareSlide, .50);
            } else {
                slideLeft(squareSlide, .50);
            }
        }
        // Now finish moving to the line
        length = (int) Math.floor(70 * StemperFiConstants.TICKS_PER_CM);
        moveBackwards(length, 0.6);
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

    // rotate the robot
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

    // Telemetry code from FRIST examples
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