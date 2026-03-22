package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;

//@Disabled
@TeleOp(group = "tests")
public class TextToSpeechTester extends ActionOpMode {

    GamepadEx gamepad;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        new GamepadButton(gamepad, GamepadKeys.Button.A).whenPressed(new Runnable() {
            @Override
            public void run() {
                telemetry.speak("I AM NOT A MORON");
                telemetry.update();
            }
        });
    }
}
