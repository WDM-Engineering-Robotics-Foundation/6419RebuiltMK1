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
        // targetVelo = Math.sqrt((dist.getSquaredNorm() * 9.806)/(dist.getNorm()*Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) - AimingConstants.ANGLE_COS_CONST));
        // targetVelo *= AimingConstants.VELO_MULT_DIST;

        // quadratic approx of velos plotted at distances: 2.10, 2.53, 3.70, 5.28
        //targetVelo = AimingConstants.VELO_DIST_A*dist.getSquaredNorm() + AimingConstants.VELO_DIST_B*dist.getNorm() + AimingConstants.VELO_DIST_C;
        targetVelo = AimingConstants.VELO_DIST_A * Math.pow(getOffsetShootingDist(), 2) + AimingConstants.VELO_DIST_B * getOffsetShootingDist() + AimingConstants.VELO_DIST_C;

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

    public static double flywheelToBallSpeed(double flywheelSpeed) {
        double rpsInverse = (-2.51619 + Math.sqrt(1.608516 * flywheelSpeed - 49.732616))/(0.804258);
        return Math.sqrt((rpsInverse * rpsInverse * 9.806)/(rpsInverse * Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) - 2 * (AimingConstants.HUB_HEIGHT.in(Units.Meters) - AimingConstants.SHOOTER_HEIGHT.in(Units.Meters)) * (Math.cos(AimingConstants.SHOOTER_ANGLE_RAD) * Math.cos(AimingConstants.SHOOTER_ANGLE_RAD))));
    }

    public static double ballSpeedToFlywheel(double ballSpeed) {
        double totalSpeed = Subsystems.drivetrain().getChassisSpeeds().vxMetersPerSecond + ballSpeed;
        double mpsInverse = (totalSpeed * totalSpeed * Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) + totalSpeed * Math.sqrt(totalSpeed * totalSpeed * Math.pow(Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD), 2) - 78.448 * (AimingConstants.HUB_HEIGHT.in(Units.Meters) - AimingConstants.SHOOTER_HEIGHT.in(Units.Meters)) * Math.pow(Math.cos(AimingConstants.SHOOTER_ANGLE_RAD), 2)))/(19.612);
        return AimingConstants.VELO_DIST_A * (mpsInverse * mpsInverse) + AimingConstants.VELO_DIST_B * mpsInverse + AimingConstants.VELO_DIST_C;
    }

    public static Translation2d getAbsoluteVelocity() {
        Translation2d relativeVelo = new Translation2d(Subsystems.drivetrain().getChassisSpeeds().vxMetersPerSecond, Subsystems.drivetrain().getChassisSpeeds().vyMetersPerSecond);
        return relativeVelo.rotateBy(Subsystems.drivetrain().getRotation3d().toRotation2d());
    }

    public static double ballSpeedVelocityY(double ballSpeed) {
        return ballSpeed * Math.cos(AimingConstants.SHOOTER_ANGLE_RAD);
    }

    public static double ballSpeedVelocityX(double ballSpeed) {
        return ballSpeed * Math.sin(AimingConstants.SHOOTER_ANGLE_RAD);
    }

    public static double getBallVelocityMeters(double pointDist) {
        return Math.sqrt((pointDist * pointDist * 9.806)/(pointDist * Math.sin(2 * AimingConstants.SHOOTER_ANGLE_RAD) - 2 * 1.10875757466 * Math.pow(Math.cos(AimingConstants.SHOOTER_ANGLE_RAD), 2)));
    }

    public static Rotation2d getAngleOffsetDegrees() {
        return Rotation2d.fromDegrees(Math.atan((ballSpeedVelocityY(flywheelToBallSpeed(targetVelo)) + Subsystems.drivetrain().getChassisSpeeds().vyMetersPerSecond)/(ballSpeedVelocityX(flywheelToBallSpeed(targetVelo)))) 
        * (180 / Math.PI) - Subsystems.drivetrain().getRotation3d().toRotation2d().getDegrees());
    }

    public static double getOffsetVelocity() {
        double shootDist = (targetPose.getX() - Subsystems.drivetrain().getState().Pose.getX())/(Math.cos(Subsystems.drivetrain().getRotation3d().toRotation2d().getRadians() - getAngleOffsetDegrees().getRadians()));
        return ballSpeedToFlywheel(getBallVelocityMeters(shootDist));
    }

    public static double getOffsetShootingDist() {
        double hubDist;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            hubDist = AimingConstants.BLUE_BASE_HUB_LOC.getX() - Subsystems.drivetrain().getState().Pose.getX();
        } else {
            hubDist = AimingConstants.BLUE_BASE_HUB_LOC.getX() - Subsystems.drivetrain().getState().Pose.getX();
            hubDist = -hubDist;
        }
        
        return hubDist / Math.cos(getAngleOffsetDegrees().getRadians());
    }
}
