package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

@TeleOp
public class Red extends LinearOpMode {


    @Override
    public void runOpMode() {
        SavedValues.teamColor = AutoShooter.TeamColor.RED;
        telemetry.addData("color", SavedValues.teamColor);
        telemetry.update();
        waitForStart();

    }
}
