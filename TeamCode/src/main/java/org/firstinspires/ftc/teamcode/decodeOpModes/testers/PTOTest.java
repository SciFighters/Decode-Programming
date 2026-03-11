package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;

@TeleOp(group = "tests")
public class PTOTest extends ActionOpMode {
    MecanumDrive mecanumDrive;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        mecanumDrive.driveMode();
        gamepad = new GamepadEx(gamepad1);
        new GamepadButton(gamepad, GamepadKeys.Button.A).whenPressed(new CommandGroups.PowerTakeOff(mecanumDrive, () -> -gamepad.getRightY()));
//        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP).whenPressed(() -> CommandGroups.PowerTakeOff.power += 0.1);
//        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> CommandGroups.PowerTakeOff.power -= 0.1);
//        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> CommandGroups.PowerTakeOff.left += 0.05);
//        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> CommandGroups.PowerTakeOff.right += 0.05);
//        new GamepadButton(gamepad, GamepadKeys.Button.B).whenPressed(() -> {
//            CommandGroups.PowerTakeOff.power = 0;
//            CommandGroups.PowerTakeOff.left = 0;
//            CommandGroups.PowerTakeOff.right = 0;
//        });
    }

    @Override
    public void run() {
        super.run();
        multipleTelemetry.addData("rightDelta",CommandGroups.PowerTakeOff.right);
        multipleTelemetry.addData("leftDelta",CommandGroups.PowerTakeOff.left);
        multipleTelemetry.addData("pitch",mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        multipleTelemetry.addData("roll",mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        multipleTelemetry.update();
    }

    @Override
    public void end() {
        mecanumDrive.driveMode();
    }
}
