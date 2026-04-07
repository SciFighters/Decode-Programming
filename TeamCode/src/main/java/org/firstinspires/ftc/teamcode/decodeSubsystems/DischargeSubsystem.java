package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DischargeSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;
    public final MotorEx flyWheelMotor;
    private final Servo rampServo;
    private final double kS = 0.17, kV = 0.00025772193, kP = 0.00133333, kI = 0.000005;
    private final double ticksPerDegree = 383.6 * (198.0 / 49.0) / 360.0;
    private double startAngle;
    private final double gearRatio = 37.0 / 37; // 34:30
    double integral;
    public static boolean shooting = false;

    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm, "flyWheelMotor", Motor.GoBILDA.BARE);
        flyWheelMotor.encoder.setDirection(Motor.Direction.FORWARD);
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
//        turretMotor.setPower(-flyWheelPower);
    }

    public void setFlyWheelRPM(double rpm) {
        if (Math.abs(rpm) < 200) {
            flyWheelMotor.set(0);
//            turretMotor.setPower(0);
            return;
        }


//        rpm *= gearRatio;
        double currentRPM = getRPM();
        if (currentRPM - rpm > 280) {
            flyWheelMotor.set(-1);
        } else if (currentRPM < rpm || shooting) {
            flyWheelMotor.set(1);
//            turretMotor.setPower(-1);
        } else {
            flyWheelMotor.set(kS * Math.signum(rpm) + kV * rpm - 0.07 + kP * (rpm - getRPM()));
//            turretMotor.setPower(-kS * Math.signum(rpm) - kV * rpm + 0.04);
        }

    }

    public void stayRPM(double rpm) {
        flyWheelMotor.set(kS * Math.signum(rpm) + kV * rpm - 0.02 + kP * (rpm - getRPM()));
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
