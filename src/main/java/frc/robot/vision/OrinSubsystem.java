package frc.robot.vision;


import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Subsystems;

public class OrinSubsystem extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable orinTable = inst.getTable("vision");

    private final StructSubscriber<Pose3d> poseEstimateSubcriber = orinTable.getStructTopic("vision_pose_estimate", Pose3d.struct).subscribe(null);

    private final StructPublisher<Pose3d> fusedPosePublisher = orinTable.getStructTopic("fused_pose_estimate", Pose3d.struct).publish();

    private final DoubleSubscriber rosTime = orinTable.getDoubleTopic("time").subscribe(-1);

    private final StructArrayPublisher<Pose2d> latestResultsPub = orinTable.getStructArrayTopic("Latest Results", Pose2d.struct).publish();

    private static final int NUM_RESULTS_STORED = 10;

    private static final Pose2d NO_RESULT = new Pose2d(Double.NaN, Double.NaN, Rotation2d.kZero);

    private final Pose2d[] latestResults = new Pose2d[NUM_RESULTS_STORED];

    private int resultIndex = -1;
    public OrinSubsystem() {
    }

    @Override
    public void periodic() {
        TimestampedObject<Pose3d>[] unreadResults = poseEstimateSubcriber.readQueue();
        if (unreadResults.length == 0) {
            latestResults[resultIndex = ((resultIndex+1) % NUM_RESULTS_STORED)] = NO_RESULT;
        }
        for (TimestampedObject<Pose3d> estimate : unreadResults) {
            //System.out.println("Adding estimate");
            latestResults[resultIndex = ((resultIndex+1) % NUM_RESULTS_STORED)] = estimate.value.toPose2d();
            Subsystems.drivetrain().addVisionMeasurement(estimate.value.toPose2d(), estimate.timestamp/1e6f, VecBuilder.fill(0.1,0.1,0.3));
        }
        
        // TimestampedObject<Pose3d> result = poseEstimateSubcriber.getAtomic();
        
        // if (result != null) {
        //     //System.out.println(result.serverTime + " : " + result.timestamp);
        //     Subsystems.drivetrain().addVisionMeasurement(result.value.toPose2d(), result.timestamp/1e6f);
        // }
        SwerveDriveState latestState = Subsystems.drivetrain().getState();
        fusedPosePublisher.set(new Pose3d(latestState.Pose), (long) (latestState.Timestamp*1e6));
        latestResultsPub.set(latestResults);
    }


}
