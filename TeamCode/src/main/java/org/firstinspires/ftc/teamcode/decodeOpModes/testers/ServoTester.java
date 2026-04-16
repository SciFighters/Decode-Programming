package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@TeleOp(group = "tests")
public class ServoTester extends ActionOpMode {
    Servo servo1, servo2;
    GamepadEx gamepad;
    GamepadButton A, B, Y, X;
    double pos = 0.5;

    @Override
    public void initialize() {
        servo1 = hardwareMap.servo.get("liftLeft");
//        servo1 = hardwareMap.get(Servo.class, "leftPTO");;
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        A.whenPressed(() -> pos += 0.01);
        B.whenPressed(() -> pos -= 0.01);
        X.whenPressed(() -> pos -= 0.1);
        Y.whenPressed(() -> pos += 0.1);

    }

    @Override
    public void run() {
        super.run();
//        servo1.setPosition(pos);
        servo1.setPosition(pos);
        multipleTelemetry.addData("pos", pos);
        multipleTelemetry.update();
    }
}
