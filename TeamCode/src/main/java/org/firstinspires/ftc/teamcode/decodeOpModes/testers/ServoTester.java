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
    GamepadButton A, B;
    double pos = 0.5;

    @Override
    public void initialize() {
        servo1 = hardwareMap.servo.get("intakeServo1");
        servo2 = hardwareMap.servo.get("intakeServo1");
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        A.whenPressed(() -> pos += 0.01);
        B.whenPressed(() -> pos -= 0.01);
    }

    @Override
    public void run() {
        super.run();
        servo1.setPosition(pos);
//        servo2.setPosition(1 - pos);
        multipleTelemetry.addData("pos",pos);
        multipleTelemetry.update();
    }
}
