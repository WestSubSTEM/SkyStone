package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Foundation Bridge", group = "Qual")
public class RedFoundationBridge extends BlueFoundationWall {

    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = false;
        isBridge = true;
        super.runOpMode();
    }
}