package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class DischargeSubsystem extends SubsystemBase {

//    private final DcMotorEx turretMotor;
    public final MotorEx flyWheelMotor;
    private final Servo rampServo;
    private final double kS = 0.14, kV = 0.00023424689, kP = 0.000833333;
    private final double ticksPerDegree = 26;

    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = new MotorEx(hm,"flyWheelMotor", Motor.GoBILDA.BARE);
//        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampServo = hm.get(Servo.class, "rampServo");
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.set(flyWheelPower);
    }
    public void setFlyWheelRPM(double rpm){
        flyWheelMotor.set(kS * Math.signum(rpm) + kV * rpm + kP * (rpm - flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60));
    }
    public double getRPM(){
        return flyWheelMotor.getVelocity() / flyWheelMotor.getCPR() * 60;
    }
    public void setTurretPower(double turretPower) {
//        turretMotor.setPower(turretPower);
    }

    public double getTurretPosition() {
//        return turretMotor.getCurrentPosition();
        return 0;
    }
    public double getTurretAngle(){
//        return turretMotor.getCurrentPosition / ticksPerDegree;
        return 0;
    }


    public void setRampDegree(double rampDegree) {
        double pos = -(34.13 - rampDegree)/(38.87);
        rampServo.setPosition(pos);
    }
}
