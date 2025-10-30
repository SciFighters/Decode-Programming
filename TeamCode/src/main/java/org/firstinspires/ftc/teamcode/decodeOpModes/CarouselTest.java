package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

@TeleOp(name = "CarouselTest",group = "tests")
public class CarouselTest extends ActionOpMode {
    CarouselSubsystem carouselSubsystem;
    IntakeSubsystem intakeSubsystem;
    GamepadEx gamepad;
    Button A,B,X,Y, rightStickButton;
    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        rightStickButton = new GamepadButton(gamepad,GamepadKeys.Button.RIGHT_STICK_BUTTON);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        CarouselCommands.MoveToPos intakePos = new CarouselCommands.MoveToPos(carouselSubsystem, carouselSubsystem.spinConversion);
        CarouselCommands.Discharge discharge = new CarouselCommands.Discharge(carouselSubsystem);
        IntakeCommands.IntakeState intakeState = new IntakeCommands.IntakeState(intakeSubsystem);
        IntakeCommands.TransferState transferState = new IntakeCommands.TransferState(intakeSubsystem);
        IntakeCommands.ClosedState closedState = new IntakeCommands.ClosedState(intakeSubsystem);

        rightStickButton.whenPressed(intakePos);
        A.whenPressed(intakeState);
        B.whenPressed(transferState);
        X.whenPressed(closedState);
        Y.whenPressed(discharge);

    }

    @Override
    public void run() {
        multipleTelemetry.addData("pos", carouselSubsystem.getPosition());
        multipleTelemetry.update();
        super.run();
    }
}
