package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Red Foundation Wall", group = "Qual")
public class RedFoundationWall extends BlueFoundationWall {

    @Override
    public void runOpMode() throws InterruptedException {
        isBlue = false;
        isBridge = false;
        super.runOpMode();
    }
}