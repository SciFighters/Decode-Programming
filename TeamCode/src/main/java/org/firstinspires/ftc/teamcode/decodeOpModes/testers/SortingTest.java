package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class SortingTest extends ActionOpMode {
    CarouselSubsystem carouselSubsystem;
    IntakeSubsystem intakeSubsystem;
    MecanumDrive mecanumDrive;
    GamepadEx gamepad;
    Button A, B, X, Y, UP;

    @Override
    public void initialize() {

        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(63, 0, -Math.PI));
        carouselSubsystem.resetEncoders();
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        UP = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);

        A.whenPressed(new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem));
        UP.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        schedule(new MecanumCommands.Drive(mecanumDrive, () -> gamepad.getLeftY(), () -> gamepad.getLeftX(), () -> gamepad.getRightX()));
    }

    @Override
    public void run() {
        multipleTelemetry.addData("greenPos", carouselSubsystem.getGreenPlacement());
        multipleTelemetry.addData("intakeCurrent", intakeSubsystem.getCurrent());
        multipleTelemetry.update();
        super.run();
    }
}
