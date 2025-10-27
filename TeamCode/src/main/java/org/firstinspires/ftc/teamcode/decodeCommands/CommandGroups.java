package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

public class CommandGroups {
    public static class Shoot extends SequentialCommandGroup {
        public Shoot(DischargeSubsystem dischargeSubsystem, IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(
                    new ParallelCommandGroup(
                            new IntakeCommands.TransferState(intakeSubsystem),
                            new DischargeCommands.setState(dischargeSubsystem, 3200, 0, 54)
                    ),
                    new CarouselCommands.Discharge(carouselSubsystem)
            );
        }
    }
}
