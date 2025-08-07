package org.firstinspires.ftc.teamcode.needle.commands;

import org.firstinspires.ftc.teamcode.actions.groups.ParallelActionGroup;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

public class Groups {
    public static ParallelActionGroup GoToBasketCmd(TelescopicArmSubsystem telescopicArmSubsystem,
                                                    ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem){
        return new ParallelActionGroup(new TelescopicArmCommands.GoTo(telescopicArmSubsystem, telescopicArmSubsystem.basketPos),
                new ArmAxisCommands.AxisGoTo(armAxisSubsystem,armAxisSubsystem.basketAngle),
                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.basketTilt));
    }
    public static ParallelActionGroup intakePos(TelescopicArmSubsystem telescopicArmSubsystem,
                                                    ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem){
        return new ParallelActionGroup(new TelescopicArmCommands.GoHome(telescopicArmSubsystem),
                new ArmAxisCommands.AxisGoTo(armAxisSubsystem,0),
                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.intakePos));
    }

}
