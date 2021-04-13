package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig forwardConfigFast =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig forwardConfigSlow =
            new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                    Constants.kMinAcclerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseConfigSlow =
            new TrajectoryConfig(Constants.kMinSpeedMetersPerSecond,
                    Constants.kMinAcclerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    TrajectoryConfig reverseConfigFast =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

        public Trajectory getDriveStraight(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    forwardConfigFast
            );
        }

        public Trajectory getDriveStraightReversed(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    reverseConfigFast
            );
        }

        //Start 8 Ball Trench Auto
        public Trajectory getCenterStartToEndOfTrench() {
        Trajectory centerStartToEndOfTrench;
        centerStartToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(40)),
                        new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                        new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(63))
                ),
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(65.66), new Rotation2d(0)),
                // Pass config
                forwardConfigSlow
        );
        return centerStartToEndOfTrench;

    }

    public Trajectory getSlalomStartToTrenchStart() {
        Trajectory slalomStartToTrenchStart;
        slalomStartToTrenchStart = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
        List.of(
                new Translation2d(Units.inchesToMeters(53), Units.inchesToMeters(39)),
                new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(85)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(85)),
                new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(85))
                // new Translation2d(Units.inchesToMeters(249), Units.inchesToMeters(33)),
                // new Translation2d(Units.inchesToMeters(284), Units.inchesToMeters(-14)),
                // new Translation2d(Units.inchesToMeters(296), Units.inchesToMeters(57)),
                // new Translation2d(Units.inchesToMeters(243), Units.inchesToMeters(47)),
                // new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(-25)),
                // new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-25)),
                // new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(-25)),
                // new Translation2d(Units.inchesToMeters(20), Units.inchesToMeters(19))
        ), 
        new Pose2d(Units.inchesToMeters(249), Units.inchesToMeters(33), Rotation2d.fromDegrees(0)),
        forwardConfigFast
        );
        return slalomStartToTrenchStart;
    }

//     public Trajectory getTrenchStartToTrenchEndA() {
//         Trajectory trenchStartToTrenchEndA;
//         trenchStartToTrenchEndA = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(Units.inchesToMeters(199), Units.inchesToMeters(40), new Rotation2d(0)), 
//                 List.of(
//                         new Translation2d(Units.inchesToMeters(235), Units.inchesToMeters(-5))
//                 ), 
//                 new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(35), Rotation2d.fromDegrees(155)), 
//                 forwardConfigSlow
//                 );
//                 return trenchStartToTrenchEndA;
//     }

//     public Trajectory getTrenchStartToTrenchEndB() {
//         Trajectory trenchStartToTrenchEndB;
//         trenchStartToTrenchEndB = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
//                 new Pose2d(Units.inchesToMeters(250), Units.inchesToMeters(35), Rotation2d.fromDegrees(155)), 
//                 List.of(
//                 ), 
//                 new Pose2d(Units.inchesToMeters(183), Units.inchesToMeters(-10), Rotation2d.fromDegrees(180)), 
//                 forwardConfigSlow
//                 );
//                 return trenchStartToTrenchEndB;
//     }

    public Trajectory getTrenchStartToSpin() {
        Trajectory trenchStartToSpin;
        trenchStartToSpin = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(83), Units.inchesToMeters(77), Rotation2d.fromDegrees(0)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(217), Units.inchesToMeters(77)),
                        new Translation2d(Units.inchesToMeters(249), Units.inchesToMeters(33)),
                        new Translation2d(Units.inchesToMeters(284), Units.inchesToMeters(-14)),
                        new Translation2d(Units.inchesToMeters(286), Units.inchesToMeters(57)),
                        new Translation2d(Units.inchesToMeters(243), Units.inchesToMeters(47))
                ), 
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(-25), Rotation2d.fromDegrees(180)), 
                forwardConfigFast
                );
                return trenchStartToSpin;
    }

    public Trajectory getTrenchEndToTrenchStart() {
        Trajectory trenchEndToTrenchStart;
        trenchEndToTrenchStart = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(245), Units.inchesToMeters(53), Rotation2d.fromDegrees(180)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(183), Units.inchesToMeters(-20)),
                        new Translation2d(Units.inchesToMeters(144), Units.inchesToMeters(-20)),
                        new Translation2d(Units.inchesToMeters(94), Units.inchesToMeters(-20))
                ), 
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), Rotation2d.fromDegrees(180)), 
                forwardConfigFast
                );
                return trenchEndToTrenchStart;
    }

    public Trajectory getBounceStartToFirstBall() {
        Trajectory bounceStartToFirstBall;
        bounceStartToFirstBall = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(62), Units.inchesToMeters(20))
                ), 
                new Pose2d(Units.inchesToMeters(65), Units.inchesToMeters(72), Rotation2d.fromDegrees(90)), 
                forwardConfigFast
                );
                return bounceStartToFirstBall;
    }

    
    public Trajectory getSecondBallToThirdTrench() {
        Trajectory secondBallToThirdTrench;
        secondBallToThirdTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(160), Units.inchesToMeters(68), Rotation2d.fromDegrees(270)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(173), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(164), Units.inchesToMeters(-75)),
                        new Translation2d(Units.inchesToMeters(222), Units.inchesToMeters(-80))
                ), 
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(84), Rotation2d.fromDegrees(90)), 
                forwardConfigFast
                );
                return secondBallToThirdTrench;
    }

    public Trajectory getThirdBallToFinalPose() {
        Trajectory thirdBallToFinalPose;
        thirdBallToFinalPose = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(240), Units.inchesToMeters(70), Rotation2d.fromDegrees(90)), 
                List.of(
                ), 
                new Pose2d(Units.inchesToMeters(290), Units.inchesToMeters(-20), Rotation2d.fromDegrees(180)), 
                reverseConfigFast
                );
                return thirdBallToFinalPose;
    }

    public Trajectory getFirstBallToSecondTrench() {
        Trajectory firstBallToSecondTrench;
        firstBallToSecondTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(77), Units.inchesToMeters(67), Rotation2d.fromDegrees(90)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(73), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(85), Units.inchesToMeters(-75)),
                        new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(-107)),
                        new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(-65))
                ), 
                new Pose2d(Units.inchesToMeters(175), Units.inchesToMeters(72), Rotation2d.fromDegrees(270)), 
                reverseConfigFast
                );
                return firstBallToSecondTrench;
    }

    public Trajectory getBounceToFinalPoseA() {
        Trajectory bounceToFinalPoseA;
        bounceToFinalPoseA = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(74), Units.inchesToMeters(0), Rotation2d.fromDegrees(150)), 
                List.of(
                ), 
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(30), Rotation2d.fromDegrees(180)), 
                forwardConfigFast
                );
                return bounceToFinalPoseA;
    }

    public Trajectory getBounceToFinalPoseB() {
        Trajectory bounceToFinalPoseB;
        bounceToFinalPoseB = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(30), Rotation2d.fromDegrees(180)), 
                List.of(
                ), 
                new Pose2d(Units.inchesToMeters(20), Units.inchesToMeters(30), Rotation2d.fromDegrees(180)), 
                forwardConfigSlow
                );
                return bounceToFinalPoseB;
    }

    public Trajectory getSlalomToFinalPose() {
        Trajectory slalomToFinalPose;
        slalomToFinalPose = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(-25), Rotation2d.fromDegrees(180)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(-25)),
                        new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(-25)),
                        new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(-25)),
                        new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(19))
                ), 
                new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(68), Rotation2d.fromDegrees(180)), 
                forwardConfigFast
                );
                return slalomToFinalPose;
    }

    public Trajectory getHyperStartToSecondTurn() {
        Trajectory hyperStartToSecondTurn;
        hyperStartToSecondTurn = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(165), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(-60)),
                        new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(-100)),
                        new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(-90)),
                        new Translation2d(Units.inchesToMeters(70), Units.inchesToMeters(-80)),
                        // new Translation2d(Units.inchesToMeters(35), Units.inchesToMeters(-30)),
                        new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(0)),
                        new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(10)),
                        new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(10)),
                        new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(20)),
                        // new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(45)),
                        new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(80)),
                        new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(80)),
                        new Translation2d(Units.inchesToMeters(130), Units.inchesToMeters(80)),
                        new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(20)),
                        new Translation2d(Units.inchesToMeters(245), Units.inchesToMeters(-55)),
                        new Translation2d(Units.inchesToMeters(300), Units.inchesToMeters(-55)),
                        new Translation2d(Units.inchesToMeters(260), Units.inchesToMeters(20)),
                        new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(20)),
                        new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(20))
                //         new Translation2d(Units.inchesToMeters(220), Units.inchesToMeters(20)),
                //         new Translation2d(Units.inchesToMeters(245), Units.inchesToMeters(20))
                ), 
                new Pose2d(Units.inchesToMeters(-40), Units.inchesToMeters(20), Rotation2d.fromDegrees(180)), 
                forwardConfigFast
                );
                return hyperStartToSecondTurn;
    }

    public Trajectory getHyperSecondTurnToThirdTurn() {
        Trajectory hyperSecondTurnToThirdTurn;
        hyperSecondTurnToThirdTurn = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(10), Rotation2d.fromDegrees(0)), 
                List.of(
                        new Translation2d(Units.inchesToMeters(250), Units.inchesToMeters(15))
                ), 
                new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(40), Rotation2d.fromDegrees(-105)), 
                forwardConfigFast
                );
                return hyperSecondTurnToThirdTurn;
    }

    public Trajectory getEndOfTrenchToStartOfTrench() {
            Trajectory endOfTrenchToStartOfTrench;
            endOfTrenchToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(65.66), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(64))
                    ),
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(50), new Rotation2d(0)),
                    // Pass config
                    reverseConfigFast
            );
            return endOfTrenchToStartOfTrench;
        }
        //End 8 Ball Trench Auto

        //Start 5 Ball Steal Auto
        public Trajectory getStealStartToStealBall() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(0), new Rotation2d(0)),
                    // Pass config
                    forwardConfigSlow
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealStartToStealBallV2() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                List.of(
                        new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(-19))
                ),
                new Pose2d(Units.inchesToMeters(139), Units.inchesToMeters(-55), Rotation2d.fromDegrees(-45.0)),
                // Pass config
                forwardConfigSlow
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealStartToStealBallV3() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(126), Units.inchesToMeters(-245), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(201), Units.inchesToMeters(-264))
                    ),
                    new Pose2d(Units.inchesToMeters(265), Units.inchesToMeters(-290), Rotation2d.fromDegrees(-45.0)),
                    // Pass config
                    forwardConfigSlow
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealBallToCenterShot() {
            Trajectory stealSpotToCenterShot;
            stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(190), new Rotation2d(0)),
                    // Pass config
                    reverseConfigFast
            );
            return stealSpotToCenterShot;
        }

    public Trajectory getStealBallToCenterShotV2() {
        Trajectory stealSpotToCenterShot;
        stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(139), Units.inchesToMeters(-55), new Rotation2d(-45)),
                List.of(
                        new Translation2d(Units.inchesToMeters(107), Units.inchesToMeters(5))
                ),
                new Pose2d(Units.inchesToMeters(56), Units.inchesToMeters(125), new Rotation2d(-70)),
                // Pass config
                reverseConfigFast
        );
        return stealSpotToCenterShot;
    }
        //End 5 Ball Steal Auto

        //Start 10 Ball Rendezvous/Trench Auto
        public Trajectory getCenterStartToRendezvous2ball() {
            Trajectory centerStartToRendezvous2Ball;
            centerStartToRendezvous2Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(40))
                    ),
                    new Pose2d(Units.inchesToMeters(155), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    // Pass config
                    forwardConfigSlow
            );
            return centerStartToRendezvous2Ball;
        }

        public Trajectory getRendezvous2BallToStartOfTrench() {
            Trajectory rendezvous2BallToStartOfTrench;
            rendezvous2BallToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(155), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(30))
                    ),
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfigSlow
            );
            return rendezvous2BallToStartOfTrench;
        }

        public Trajectory getStartOfTrenchToEndOfTrench() {
            Trajectory startOfTrenchToEndOfTrench;
            startOfTrenchToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(65.66))
                    ),
                    new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(65.66), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    forwardConfigSlow
            );
            return startOfTrenchToEndOfTrench;
        }

        //Start 3 Ball Safe Auto
        public Trajectory getLeftStartToSafe(){
            Trajectory leftStartToSafe;
            leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(-60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfigFast
            );
            return leftStartToSafe;
        }

        public Trajectory getLeftStartToSafeTest() {
                Trajectory leftStartToSafe;
                leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                        List.of(
                                new Translation2d(Units.inchesToMeters(-50), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-150), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-200), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-250), Units.inchesToMeters(0))
                        ),
                        new Pose2d(Units.inchesToMeters(-300), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                        // Pass config
                        reverseConfigSlow
                );
                return leftStartToSafe;
            }

        public Trajectory getSafeToLeftStartTest() {
                Trajectory safeToLeftStart;
                safeToLeftStart = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                        new Pose2d(Units.inchesToMeters(-300), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                        List.of(
                                new Translation2d(Units.inchesToMeters(-250), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-200), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-150), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(-50), Units.inchesToMeters(0))
                        ),
                        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                        // Pass config
                        forwardConfigFast
                );
                return safeToLeftStart;
            }

    public Trajectory getLeftStartToSafeForward(){
        Trajectory leftStartToSafe;
        leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
                ),
                new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                // Pass config
                forwardConfigFast
        );
        return leftStartToSafe;
    }
    public Trajectory getStartToTrenchStart(){
        Trajectory startToTrenchStart;
        startToTrenchStart = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                        new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))
                ),
                new Pose2d(Units.inchesToMeters(60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                // Pass config
                forwardConfigFast
        );
        return startToTrenchStart;
    }
        //End 3 Ball Safe Auto
    }
