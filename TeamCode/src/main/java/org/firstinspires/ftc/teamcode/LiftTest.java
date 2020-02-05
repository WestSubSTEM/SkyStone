package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.spa.ftclib.internal.state.Button;

/**
 * Created by Gabriel on 2018-08-05.
 * Simple Button demo that spins a motor for 1 second when the X button on gamepad 1 is pressed
 * Also useful as a simple motor test
 * Tested and found fully functional by Gabriel on 2018-8-5.
 */

@Disabled
@TeleOp(name = "Lift Test", group = "sample")

public class LiftTest extends OpMode {
    private Button button;
    private ElapsedTime timer;
    private boolean spinning = false;
    private DcMotor liftLeftMotor;
    private DcMotor liftMiddleMotor;
    private DcMotor liftRightMotor;
    private DcMotor[] liftMotors;
    private int[] liftPositions = new int[] {0, 1391, 3245, 5099, 6953, 8807, 10662, 12516, 14370, 16224, 18078, 19933};

    @Override
    public void init() {

        button = new Button();


        timer = new ElapsedTime();
        liftLeftMotor = hardwareMap.get(DcMotor.class, "liftLeftMotor");
        liftMiddleMotor = hardwareMap.get(DcMotor.class, "liftMiddleMotor");
        liftRightMotor = hardwareMap.get(DcMotor.class, "liftRightMotor");
        liftMiddleMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotors = new DcMotor[3];
        liftMotors[0] = liftLeftMotor;
        liftMotors[1] = liftMiddleMotor;
        liftMotors[2] = liftRightMotor;
        for (DcMotor motor : liftMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {

        double power = (gamepad1.left_stick_y * -1);
        for (DcMotor motor : liftMotors) {
            motor.setPower(power);
        }
        telemetry.addData("p: ", power);
        telemetry.addData("l: ", liftLeftMotor.getCurrentPosition());
        telemetry.addData("c: ", liftLeftMotor.getCurrentPosition());
        telemetry.addData("r: ", liftRightMotor.getCurrentPosition());

//        button.input(gamepad1.x);
//        if (button.onPress()) {
//            timer.reset();
//            spinning = true;
//        }
//        if (spinning && timer.seconds() > 1) {
//            spinning = false;
//        }
    }
}
