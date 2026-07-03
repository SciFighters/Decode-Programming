package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DischargeSubsystem extends SubsystemBase {
    public final FlywheelSubsystem flywheel;
    public final TurretSubsystem turret;
    public final RampSubsystem ramp;
    public static boolean shooting = false;

    public DischargeSubsystem(HardwareMap hm) {
        flywheel = new FlywheelSubsystem(hm);
        turret = new TurretSubsystem(hm);
        ramp = new RampSubsystem(hm);

        shooting = false;
    }

    public void resetTurret() {
        turret.reset();
    }
}