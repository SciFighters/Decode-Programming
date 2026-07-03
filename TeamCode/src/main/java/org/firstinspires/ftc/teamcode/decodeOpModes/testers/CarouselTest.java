package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;

@TeleOp(name = "CarouselTest", group = "tests")
public class CarouselTest extends ActionOpMode {
    CarouselSubsystem carousel;
    GamepadEx gamepadEx;
    Button A, B;

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);

        carousel = new CarouselSubsystem(hardwareMap);
        carousel.resetEncoders();
        gamepadEx.getGamepadButton(GamepadKeys.Button.A).whenPressed(new CarouselCommands.RotateDistance(carousel, 3, 1));
    }

    @Override
    public void run() {
        carousel.setSpinPower(gamepad1.left_stick_x);
        multipleTelemetry.addData("pos", carousel.getPosition());
        multipleTelemetry.addData("angle", carousel.getAngle());
        multipleTelemetry.update();
        super.run();
    }
}
