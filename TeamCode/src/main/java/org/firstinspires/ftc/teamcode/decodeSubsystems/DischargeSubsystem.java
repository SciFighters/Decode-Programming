package org.firstinspires.ftc.teamcode.decodeSubsystems;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BangBangController;

public class DischargeSubsystem extends SubsystemBase {
    private final DcMotorEx turretMotor;
    public final MotorEx flyWheelMotor;
    private final Servo rampServo;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.07, 0.00016305959, 0);
    private final PIDController pid = new PIDController(0.00133333, 0.000005, 0);
    private final BangBangController bangbang = new BangBangController(200);
    private final double ticksPerDegree = 383.6 * (198.0 / 49.0) / 360.0;
    private double startAngle;
    private final double gearRatio = 37.0 / 37; // 34:30
    double integral;
    public static boolean shooting = false;

    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm, "flyWheelMotor", Motor.GoBILDA.BARE);
        flyWheelMotor.encoder.setDirection(Motor.Direction.REVERSE); // for testing 2.7.26
        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampServo = hm.get(Servo.class, "rampServo");
        startAngle = 180;
        integral = 0;
        shooting = false;
    }

    public void resetTurret() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.set(flyWheelPower);
    }

    public void setFlyWheelRPM(double rpm) {
        if (abs(rpm) < 200) {
            flyWheelMotor.set(0);
            return;
        }

        double currentRPM = getRPM();

        if (bangbang.isAtSetpoint(currentRPM, rpm)) { // switching to pid for close range-error
            flyWheelMotor.set(calculateCloseLoopOutput(currentRPM, rpm));
        } else {
            flyWheelMotor.set(bangbang.calculate(currentRPM, rpm));
        }
    }

    private double calculateCloseLoopOutput(double currentRPM, double setpoint) {
        return pid.calculate(currentRPM, setpoint) + feedforward.calculate(setpoint);
    }

    public void stayRPM(double rpm) {
        flyWheelMotor.set(calculateCloseLoopOutput(getRPM(), rpm));
    }

    public double getRPM() {
        return -flyWheelMotor.getCorrectedVelocity() / flyWheelMotor.getCPR() * 60 * gearRatio;
    }

    public double getFlyWheelPower() {
        return flyWheelMotor.get();
    }

    public void setTurretPower(double turretPower) {
        if (getTurretAngle() > 340) {
            turretPower = Math.max(turretPower, 0);
        } else if (getTurretAngle() < 20) {
            turretPower = Math.min(turretPower, 0);
        }
        turretMotor.setPower(turretPower);
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getTurretAngle() {
        return 360 - (turretMotor.getCurrentPosition() / ticksPerDegree + startAngle);
    }

    public void setTurretAngle(double angle) {
        startAngle += getTurretAngle() - angle;
    }

    public double getRPS() {
        return turretMotor.getVelocity(AngleUnit.RADIANS) / (198.0 / 49.0);
    }

    public void setRampDegree(double rampDegree) {
        double pos = (60 - rampDegree) / 30.0;
        rampServo.setPosition(Math.min(pos * 0.6 - 0.2, 0.6));
    }
}