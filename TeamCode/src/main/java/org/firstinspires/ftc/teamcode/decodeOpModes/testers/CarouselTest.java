package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

@TeleOp(name = "CarouselTest", group = "tests")
public class CarouselTest extends ActionOpMode {
    CarouselSubsystem carouselSubsystem;
    GamepadEx gamepadEx;
    Button A, B;

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);

        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new CarouselCommands.SlideDistance(carouselSubsystem,3,1));
    }

    @Override
    public void run() {
        carouselSubsystem.setSpinPower(gamepad1.left_stick_x);
        multipleTelemetry.addData("pos", carouselSubsystem.getPosition());
        multipleTelemetry.addData("angle", carouselSubsystem.getAngle());
        multipleTelemetry.update();
        super.run();
    }
}
