package org.firstinspires.ftc.teamcode.needle.tuning;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.needle.commands.TelescopicArmCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;
@TeleOp
public class TelescopicArmTuner extends ActionOpMode {
    TelescopicArmSubsystem telescopicArmSubsystem;
    GamepadEx gamepad;
    Button a,b;


    @Override
    public void initialize() {
        telescopicArmSubsystem = new TelescopicArmSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        a = new GamepadButton(gamepad, GamepadKeys.Button.A);
        b = new GamepadButton(gamepad, GamepadKeys.Button.B);
        telescopicArmSubsystem.setDefaultCommand(new TelescopicArmCommands.ExtensionManual(telescopicArmSubsystem, gamepad::getLeftY));
        a.whenPressed(new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 60));
        b.whenPressed(new TelescopicArmCommands.GoHome(telescopicArmSubsystem));
    }

    @Override
    public void run() {
        super.run();
        multipleTelemetry.addData("cm",telescopicArmSubsystem.getPosition());
        multipleTelemetry.addData("current", telescopicArmSubsystem.getCurrent());
        multipleTelemetry.update();
    }
}
