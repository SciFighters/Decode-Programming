package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@TeleOp(group = "tests")
public class ServoTester extends ActionOpMode {
    Servo servo;
    GamepadEx gamepad;
    GamepadButton A, B;

    @Override
    public void initialize() {
        servo = hardwareMap.servo.get("servo");
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        A.whenPressed(() -> servo.setPosition(1));
        B.whenPressed(() -> servo.setPosition(0));
    }


}
