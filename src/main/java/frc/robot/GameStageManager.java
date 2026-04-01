package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GameStageManager {

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
    private static final NetworkTable stateTable = inst.getTable("Game State");

    private static final DoublePublisher transitionTime = stateTable.getDoubleTopic("Transition Time").publish();

    public static void update() {
        
    }
}
