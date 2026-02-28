package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AimingConstants;
import frc.robot.subsystems.Subsystems;

import static frc.robot.Constants.AimingConstants;

public class FieldCalculations {

    

    private static Translation2d targetPose = Translation2d.kZero;

    private static double deltaHubDistance = 0;

    private static double robotVelo = 0;

    private static double targetVelo = 0;

    private static double travelTime = 0;

    public static void update() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            targetPose = AimingConstants.BLUE_BASE_HUB_LOC;
        } else {
            targetPose = AimingConstants.RED_BASE_HUB_LOC;
        }

        ChassisSpeeds speeds = Subsystems.drivetrain().getChassisSpeeds();

        Translation2d dist = Subsystems.drivetrain().getState().Pose.getTranslation().minus(targetPose);

        double heightDiff = AimingConstants.HUB_HEIGHT.minus(AimingConstants.SHOOTER_HEIGHT).in(Units.Meters);

        

        // https://www.desmos.com/calculator/4lntym3xnv
        targetVelo = Math.sqrt((dist.getSquaredNorm() * 9.806)/(dist.getNorm()*Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) - AimingConstants.ANGLE_COS_CONST));
        travelTime = (targetVelo * (Math.sin(AimingConstants.SHOOTER_ANGLE_RAD)) + Math.sqrt(Math.pow(targetVelo * (Math.sin(AimingConstants.SHOOTER_ANGLE_RAD)),2) + 19.612 * (-heightDiff)))/(9.806);
        
        double deltaX = speeds.vxMetersPerSecond;
        double deltaY = speeds.vyMetersPerSecond;

        robotVelo = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
    
        // This is the derivative of the distance between the hub and robot, assuming that the change in pos of the hub is 0.
        deltaHubDistance = 0.5 * Math.pow(dist.getX()*dist.getX() + dist.getY()*dist.getY(), -0.5) * (-2*dist.getX()*deltaX - 2*dist.getY()*deltaY);
        

        //targetPose = targetPose.rotateAround(robotPose.getTranslation(), Rotation2d.fromRadians(-speeds.omegaRadiansPerSecond*AimingConstants.VELO_MULT));
    }

    public static Translation2d getTargetPose() {
        return targetPose;
    }

    public static double robotVelo() {
        return robotVelo;
    }

    public static double getDeltaHubDist() {
        return deltaHubDistance;
    }

    public static double getBaseTargetVelo() {
        return targetVelo;
    }
}
