package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class CommandGroups {
    public static class Shoot extends ParallelCommandGroup {
        public Shoot(DischargeSubsystem dischargeSubsystem, IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(
                    new SequentialCommandGroup(
                            new IntakeCommands.TransferState(intakeSubsystem),
                            new WaitUntilCommand(() -> DischargeCommands.setState.canShoot),
                            new CarouselCommands.Discharge(carouselSubsystem)
                    ),
                    new DischargeCommands.setState(dischargeSubsystem, 3200, 0, 54)

            );
        }
    }

}
