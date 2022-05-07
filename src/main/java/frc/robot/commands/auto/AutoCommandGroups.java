package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.shooter.ShootCommandGroup;
import frc.robot.commands.shooter.ShootSequence;


public class AutoCommandGroups {
    public static final String FIELD_STRING = 
    //   ___.___.___.___.___.___.___.___.___.___.___.___.___.___.___ 
    // Red  balls:  B D E / G I L / M N
    // Blue balls:  A C F / H J K / O P
    // Red  positions: 1 2 3 
    // Blue positions: 4 5 6
        "/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///"
    + "\n//                    |_   F   [E]                   [M] //"
    + "\n//                 [G] |_                                //"
    + "\n                 H       |_ [3]       [D]                  "
    + "\n                       4   |     [2]                       "
    + "\n//                          =---=         C              //"
    + "\n//                         /   /                         //"
    + "\n//             [I]        =---=                          //"
    + "\n                        5      |_ [1]                      "
    + "\n                  J         6   |_     [B]                " 
    + "\n//                                 |_  A                 //"
    + "\n//  N                     K   [L]   |                    //"
    + "\n/// /// /// /// /// /// /// /// /// /// /// /// /// /// ///";

    // lmao
    public SequentialCommandGroup TwoBallAuto;
    public SequentialCommandGroup ThreeBallAuto;
    public SequentialCommandGroup FourBallAuto;
    public SequentialCommandGroup FiveBallAuto;

    public AutoCommandGroups (DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, String initialPos) {

        this.TwoBallAuto.addCommands(
            new ParallelCommandGroup(
                new IntakeCommandGroup(),
                drive.getRamseteCommand("Pos" + initialPos + "_2Ball")
            ),
            new ShootSequence()  
        );

        switch (initialPos) {
            case "2":
            this.FourBallAuto.addCommands(
                TwoBallAuto,
                new ParallelCommandGroup(
                    new IntakeCommandGroup(),
                    drive.getRamseteCommand("Pos" + initialPos + "_4Ball")
                ),
                new ShootSequence()  
            );
                break;
            case "3":
                break;
            
        }
    }
}
