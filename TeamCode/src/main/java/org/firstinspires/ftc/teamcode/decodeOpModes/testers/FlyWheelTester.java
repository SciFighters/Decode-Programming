package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

@Config
@TeleOp(group = "tests")
public class FlyWheelTester extends ActionOpMode {
    GamepadEx gamepad;
    GamepadButton A, B, X, Y, up, down;
    //    MotorEx flyWheelMotor;
//    Servo servo;
    double power = 0;
    double kS = 0.09, kV = 0.0002, kP = 0.000833333, kI = 0.001;
    public static double wantedRpm = 6000;
    double wantedAngle = 28;
    DischargeSubsystem dischargeSubsystem;
    DcMotor m2;

    @Override
    public void initialize() {
//        m2 = hardwareMap.dcMotor.get("turretMotor");
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        up = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
        down = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
        up.whenPressed(() -> wantedAngle += 1);
        down.whenPressed(() -> wantedAngle -= 1);
        A.whenPressed(() -> wantedRpm += 100);
        B.whenPressed(() -> wantedRpm -= 100);
//        X.whenPressed(() -> wantedAngle = 75);
//        Y.whenPressed(() -> wantedAngle = 30);
//        A.whenPressed(() -> power += 0.01);
//        B.whenPressed(() -> power -= 0.01);
        X.whenPressed(() -> power += 0.1);
        Y.whenPressed(() -> power -= 0.1);
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
//        m2.setPower(power);
        dischargeSubsystem.setRampDegree(wantedAngle);
//        dischargeSubsystem.setFlyWheelRPM(wantedRpm);
//        dischargeSubsystem.setFlyWheelPower(power);
        dischargeSubsystem.setFlyWheelRPM(wantedRpm);

        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("power", dischargeSubsystem.getFlyWheelPower());
        multipleTelemetry.addData("wanted", wantedRpm);
        multipleTelemetry.addData("angle", wantedAngle);
        multipleTelemetry.update();
        super.run();
    }


}
