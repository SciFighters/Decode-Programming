package org.firstinspires.ftc.teamcode.decodeOpModes.testers;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

@TeleOp(group = "tests")
public class TouchSensorTester extends ActionOpMode {


    double lastTime, currentTime;
    boolean last = false;
    DigitalChannel left;
    IntakeSubsystem intakeSubsystem;
    int i = 0;
    GamepadEx gamepad;
    Button A, B, X, Y;


    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        ;
        X.whenPressed(new IntakeCommands.IntakeState(intakeSubsystem));
        Y.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        B.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
    }

    @Override
    public void run() {
        super.run();

        multipleTelemetry.addData("count", IntakeSubsystem.count);
//        multipleTelemetry.addData("left",intakeSubsystem.leftSwitch.getState()  ? 0:1);
//        multipleTelemetry.addData("right",intakeSubsystem.rightSwitch.getState() ? 0:1);
        multipleTelemetry.update();
    }

    @Override
    public void end() {
        intakeSubsystem.stopSensorThread();
    }
}

