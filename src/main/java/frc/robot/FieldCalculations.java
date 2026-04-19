package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.datalog.RawLogEntry;
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

    private static Translation2d turretPose = new Translation2d();

    public static void update() {

        Pose2d robotPose = Subsystems.drivetrain().getState().Pose;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            if (robotPose.getX() > AimingConstants.BLUE_BASE_HUB_LOC.getX()) { // in neutral zone (passing)
                if (robotPose.getY() > AimingConstants.BLUE_BASE_HUB_LOC.getY()) {
                    targetPose = AimingConstants.BLUE_BASE_NORTH_PASS_LOC;
                } else {
                    targetPose = AimingConstants.BLUE_BASE_SOUTH_PASS_LOC;
                }
            } else { // in alliance zone
                targetPose = AimingConstants.BLUE_BASE_HUB_LOC;
            }
            
        } else {
            if (robotPose.getX() < AimingConstants.RED_BASE_HUB_LOC.getX()) { // in neutral zone (passing)
                if (robotPose.getY() > AimingConstants.RED_BASE_HUB_LOC.getY()) {
                    targetPose = AimingConstants.RED_BASE_NORTH_PASS_LOC;
                } else {
                    targetPose = AimingConstants.RED_BASE_SOUTH_PASS_LOC;
                }
            } else { // in alliance zone
                targetPose = AimingConstants.RED_BASE_HUB_LOC;
            }
        }

        Rotation2d robotRotation = robotPose.getRotation();

        Translation2d robotRelativeTurretPose = Constants.ShooterConstants.SHOOTER_POS_OFFSET.rotateBy(robotRotation);

        turretPose = robotPose.getTranslation().plus(robotRelativeTurretPose);

        ChassisSpeeds speeds = Subsystems.drivetrain().getChassisSpeeds();

        Translation2d dist = turretPose.minus(targetPose);

        //System.out.println(dist.getNorm());

        double distNorm = dist.getNorm();

        double timeOfFlight = AimingConstants.TOF_A*distNorm + AimingConstants.TOF_B;

        Translation2d modifiedTarget = dist;
        for (int i = 0; i < 3; i++) {
            modifiedTarget = targetPose.minus(getAbsoluteRobotVelo().times(timeOfFlight));

            dist = turretPose.minus(modifiedTarget);

            distNorm = dist.getNorm();

            timeOfFlight = AimingConstants.TOF_A*distNorm + AimingConstants.TOF_B;
        }

        targetPose = modifiedTarget;

        double heightDiff = AimingConstants.HUB_HEIGHT.minus(AimingConstants.SHOOTER_HEIGHT).in(Units.Meters);

        

        // https://www.desmos.com/calculator/4lntym3xnv
        // targetVelo = Math.sqrt((dist.getSquaredNorm() * 9.806)/(dist.getNorm()*Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) - AimingConstants.ANGLE_COS_CONST));
        // targetVelo *= AimingConstants.VELO_MULT_DIST;

        // quadratic approx of velos plotted at distances: 2.10, 2.53, 3.70, 5.28
        targetVelo = AimingConstants.VELO_DIST_A*dist.getSquaredNorm() + AimingConstants.VELO_DIST_B*dist.getNorm() + AimingConstants.VELO_DIST_C;

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

    public static Translation2d getAbsoluteRobotVelo() {
        Translation2d relativeVelo = new Translation2d(
            Subsystems.drivetrain().getChassisSpeeds().vxMetersPerSecond, 
            Subsystems.drivetrain().getChassisSpeeds().vyMetersPerSecond
        );

        return relativeVelo.rotateBy(Subsystems.drivetrain().getState().Pose.getRotation());
    }

    public static double getDeltaHubDist() {
        return deltaHubDistance;
    }

    public static double getBaseTargetVelo() {
        return targetVelo;
    }

    public static Translation2d getTurretPose() {
        return turretPose;
    }
}
