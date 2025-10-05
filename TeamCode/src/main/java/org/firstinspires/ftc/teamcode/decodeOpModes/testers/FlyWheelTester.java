package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    GamepadButton A, B, X;
    MotorEx flyWheelMotor;
//    double power = 0.00;
    double kS = 0.04, kV = 0.00015915963, kP = 0.0000833333;
    public static double wantedRpm = 1000;

    @Override
    public void initialize() {

        flyWheelMotor = new MotorEx(hardwareMap, "flyWheelMotor", Motor.GoBILDA.BARE);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        A.whenPressed(() -> wantedRpm += 500);
        B.whenPressed(() -> wantedRpm -= 500);
//        A.whenPressed(() -> flyWheelMotor.set(0.8));
//        B.whenPressed(() -> flyWheelMotor.set(0.6));
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
        flyWheelMotor.set(calculate(wantedRpm));
//        flyWheelMotor.set(gamepad.getRightY());
        multipleTelemetry.addData("power", flyWheelMotor.motorEx.getPower());
        multipleTelemetry.addData("velocity", flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60);
        multipleTelemetry.addData("correctedVelocity", flyWheelMotor.getCorrectedVelocity() / flyWheelMotor.getCPR() * 60);
        multipleTelemetry.update();
        super.run();
    }

    private double calculate(double rpm) {
        return kS * Math.signum(rpm) + kV * rpm + kP * (rpm - flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60);
    }
}
