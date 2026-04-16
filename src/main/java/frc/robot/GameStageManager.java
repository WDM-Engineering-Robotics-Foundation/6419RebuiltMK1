package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameStageManager {

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
    private static final NetworkTable stateTable = inst.getTable("Game State");

    private static final DoublePublisher transitionTime = stateTable.getDoubleTopic("Transition Time").publish();

    private static final BooleanPublisher isActive = stateTable.getBooleanTopic("Is Active").publish();

    /**
     * B = Blue
     * R = Red
     * A = All
     */
    
    private static char active = 'A';

    private static double remainingTime = 0.0;

    public static void update() {
        char firstActive = 'A';
        if (!DriverStation.getGameSpecificMessage().isEmpty()) {
            firstActive = DriverStation.getGameSpecificMessage().charAt(0);
        }
        
        // subtract transition period
        double currentTime = DriverStation.getMatchTime()-10;
        
        if (currentTime <= 0 || DriverStation.isAutonomous()) { // transition/auto
            active = 'A';
            remainingTime = -currentTime;
        } else if (currentTime >= 100) { // endgame
            active = 'A';
            remainingTime = currentTime-100;
        } else if ((currentTime/20) % 2 == 0 || firstActive == 'A') { // auto loser active
            active = firstActive;       
            remainingTime = 20 - (currentTime % 20.0);
        } else { // auto winner active
            active = firstActive == 'R' ? 'B' : 'R';
            remainingTime = 20 - (currentTime % 20.0);
        }

        isActive.set((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ^ active == 'R') || active == 'A');
    }
}
