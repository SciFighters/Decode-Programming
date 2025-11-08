package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

public class TurretTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B;
    DischargeSubsystem dischargeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
    }

    @Override
    public void run() {
        multipleTelemetry.addData("ticks",dischargeSubsystem.getTurretPosition());
        multipleTelemetry.addData("angle",dischargeSubsystem.getTurretPosition());
        multipleTelemetry.update();
        super.run();
    }
}
