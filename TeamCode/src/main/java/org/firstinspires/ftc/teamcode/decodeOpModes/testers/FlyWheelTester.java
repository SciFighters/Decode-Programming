package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@Config
@TeleOp(group = "tests")
public class FlyWheelTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B, X, up, down;
    MotorEx flyWheelMotor;
    Servo servo;
//    double power = 0.00;
    double kS = 0.09, kV = 0.0002, kP = 0.000833333;
    public static double wantedRpm = 2000;
    double currentPos = 0;

    @Override
    public void initialize() {

        flyWheelMotor = new MotorEx(hardwareMap, "flyWheelMotor", Motor.GoBILDA.BARE);
        servo = hardwareMap.servo.get("flyWheelServo");
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        up = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
        up.whenPressed(() ->currentPos += 0.1);
        down.whenPressed(() ->currentPos -= 0.1);
        A.whenPressed(() -> wantedRpm += 100);
        B.whenPressed(() -> wantedRpm -= 100);
//        A.whenPressed(() -> power += 0.01);
//        B.whenPressed(() -> power -= 0.01);
//        X.whenPressed(() -> power = 0);
//        X.whenPressed(new Runnable() {
//            @Override
//            public void run() {
//                power += 0.01;
//                flyWheelMotor.set(power);
//            }
//        });


//        flyWheelMotor.setRunMode(Motor.RunMode.VelocityControl);
    }

    @Override
    public void run() {
        servo.setPosition(currentPos);
        flyWheelMotor.set(calculate(wantedRpm));
//        flyWheelMotor.set(power);
        multipleTelemetry.addData("power", flyWheelMotor.motorEx.getPower());
        multipleTelemetry.addData("velocity", flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60);
        multipleTelemetry.addData("correctedVelocity", flyWheelMotor.getCorrectedVelocity() / flyWheelMotor.getCPR() * 60);
        multipleTelemetry.addData("servo pos", currentPos);
        multipleTelemetry.update();
        super.run();
    }

    private double calculate(double rpm) {
        return kS * Math.signum(rpm) + kV * rpm + kP * (rpm - flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60);
    }

}
