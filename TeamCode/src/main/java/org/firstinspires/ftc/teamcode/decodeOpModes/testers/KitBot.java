package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class KitBot extends ActionOpMode {
    MecanumDrive mecanumDrive;
    GamepadEx gamepad;
    Button A, B, X, Y;
    DcMotor intake;

    @Override
    public void initialize() {
        intake = hardwareMap.get(DcMotor.class,"intakeMotor");
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(63, 0, -Math.PI));

        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);

        A.whenPressed(() -> intake.setPower(1));
        B.whenPressed(() -> intake.setPower(0));
        X.whenPressed(() -> intake.setPower(0.5));
        Y.whenPressed(() -> intake.setPower(-1));

        schedule(new MecanumCommands.Drive(mecanumDrive, () -> gamepad.getLeftY(), () -> gamepad.getLeftX(), () -> gamepad.getRightX()));
    }

    @Override
    public void run() {
        super.run();
    }
}
