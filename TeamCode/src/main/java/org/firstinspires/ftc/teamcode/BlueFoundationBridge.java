package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Blue Foundation Bridge", group = "Qual")
public class BlueFoundationBridge extends BlueFoundationWall {

    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = true;
        isBridge = true;
        super.runOpMode();
    }
}