package org.firstinspires.ftc.teamcode.needle.tuning;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.needle.commands.ArmAxisCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;

public class AxisEncoderTuner extends ActionOpMode {
    ArmAxisSubsystem armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
    GamepadEx gamepad;
    Button a;
    double start = 0;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        a = new GamepadButton(gamepad, GamepadKeys.Button.A);
        armAxisSubsystem.setDefaultCommand(new ArmAxisCommands.AxisManual(armAxisSubsystem, gamepad::getRightY));
        a.whenPressed((() -> {throw new RuntimeException("A was pressed ):");}));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("pos",armAxisSubsystem.getAngle());
    }
}
