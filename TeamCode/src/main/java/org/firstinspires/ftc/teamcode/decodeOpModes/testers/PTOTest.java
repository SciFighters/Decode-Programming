package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
@TeleOp(group = "tests")
public class PTOTest extends ActionOpMode {
    MecanumDrive mecanumDrive;
    GamepadEx gamepad;
    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        mecanumDrive.driveMode();
        gamepad = new GamepadEx(gamepad1);
        new GamepadButton(gamepad, GamepadKeys.Button.A).whenPressed(new CommandGroups.PowerTakeOff(mecanumDrive,gamepad::getRightY));
    }
}
