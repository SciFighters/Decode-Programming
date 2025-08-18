package org.firstinspires.ftc.teamcode.needle.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.needle.commands.ArmAxisCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
@Config
@TeleOp
public class AxisEncoderTuner extends ActionOpMode {
    ArmAxisSubsystem armAxisSubsystem;
    GamepadEx gamepad;
    Button a,b;
    public static int angle = 90;

    @Override
    public void initialize() {
        armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        a = new GamepadButton(gamepad, GamepadKeys.Button.A);
        b = new GamepadButton(gamepad, GamepadKeys.Button.B);
        armAxisSubsystem.setDefaultCommand(new ArmAxisCommands.AxisManual(armAxisSubsystem, gamepad::getLeftY));
        a.whenPressed(new ArmAxisCommands.AxisGoTo(armAxisSubsystem, angle));
        b.whenPressed(new ArmAxisCommands.AxisGoTo(armAxisSubsystem,0));
    }

    @Override
    public void run() {
        super.run();
        multipleTelemetry.addData("pos",armAxisSubsystem.getAngle());
        multipleTelemetry.addData("current",armAxisSubsystem.getCurrent());
        multipleTelemetry.update();
    }
}
