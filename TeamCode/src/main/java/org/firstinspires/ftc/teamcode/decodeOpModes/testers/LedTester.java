package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@TeleOp(group = "tests")
public class LedTester extends ActionOpMode {
    RevBlinkinLedDriver ledDriver;
    GamepadEx gamepad;
    GamepadButton A, B, Y, X;


    @Override
    public void initialize() {
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        A.whenPressed(() -> ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE));
        B.whenPressed(() -> ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE));
        X.whenPressed(() -> ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE));
        Y.whenPressed(() -> ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE));

    }

    @Override
    public void run() {
        super.run();


    }
}

