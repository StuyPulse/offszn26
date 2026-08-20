<div align="center">
    <img src="/assets/logos/StuyPulseLogo.png" width="180" />
    <h1>TBD 2026</h1>
    <p>FRC Team StuyPlus <b>694</b> - Offseason Bot for the 2026 Rebuilt season</p>
</div>
<div align="center">

[![Java Version](https://img.shields.io/badge/Java-17-F29111?style=for-the-badge&logo=openjdk)](https://github.com/StuyPulse/offszn26/tree/main/src/main/java/com/stuypulse/robot)
[![License](https://img.shields.io/badge/License-MIT-750014?style=for-the-badge&logo=markdown)](LICENSE)
![Build](https://img.shields.io/github/actions/workflow/status/StuyPulse/offszn26/gradle.yml?style=for-the-badge&label=Build)
<br>
</div>

---

## Table of Contents
- [Inspiration](#based-on-2910s-re-blitz-for-rebuilt)
- [Branch Naming Convention](#branch-naming-convention)
- Subsystems
    - [Drivetrain](#drivetrain)
    - [Intake](#intake)
    - [Feeder](#feeder)
    - [Shooter](#shooter)
    - [Vision](#vision)
- [License](#license)

## Based on [2910's Re-Blitz](https://www.chiefdelphi.com/t/2910-cad-and-tech-binder-release-2026/521705) for Rebuilt

[![WCP 2026 Rebuilt Competitive Concept Video](https://www.video-thumbnail.com/youtube/1rzbSWugDUQ)](https://www.youtube.com/watch?v=1rzbSWugDUQ)

## Branch Naming Convention
| Prefix | Use Case |
|--------|---------|
| `main` | Production branch. |
| `feat/` | Development of new features. |
| `fix/` | Investigation of unresolved bugs that need more debugging and experimentation. |
| `experiments/` | Experimental work that may not be merged into to `main`. |
| `versions` | Different variants of the code for specific events |

## Drivetrain
**POIs**: 
- [`src/main/java/com/stuypulse/robot/subsystems/swerve`](src/main/java/com/stuypulse/robot/subsystems/swerve)
- [`src/main/java/com/stuypulse/robot/util/swerve/swerveinput/DriveInputProcessor.java`](src/main/java/com/stuypulse/robot/util/swerve/DriveInputProcessor.java)
- [`src/main/java/com/stuypulse/robot/util/swerve/swerveinput/DriveTurnInputProcessor.java`](src/main/java/com/stuypulse/robot/util/swerve/DriveTurnInputProcessor.java)

We utilize a raised swerve drivetrain with CTRE Phoenix 6 hardware and four swerve modules, letting us independently rotate the wheels. This allows for higher manueverability.

The robot is driven `Field-Centric`.

We use both [Vision](#vision) and odometry for pose estimation. Check the vision section for details. 

For driver input, we created two classes called `DriveInputProcessor.java` and `DriveTurnInputProcessor.java`. These take in driver input and processes it, applying a deadband, power curve, clamp to magnitude 1, scaling to max velocity, rate limit, and a low pass filter. This allows us to better handle controller input by letting us apply maximum velocity and maximum acceleration.

## Intake
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/intake`](src/main/java/com/stuypulse/robot/subsystems/intake)

## Feeder
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/feeder`](src/main/java/com/stuypulse/robot/subsystems/feeder)

## Shooter
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/shooter`](src/main/java/com/stuypulse/robot/subsystems/shooter)

## Indexer
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/indexer`](src/main/java/com/stuypulse/robot/subsystems/indexer)

## Hood
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/hood`](src/main/java/com/stuypulse/robot/subsystems/hood)

## Vision
**POI**: [`src/main/java/com/stuypulse/robot/subsystems/vision`](src/main/java/com/stuypulse/robot/subsystems/vision), 

This uses Limelights from [Limelight Vision](https://limelightvision.io/) to use the AprilTags to estimate the pose of the robot. It works in combination with odometry to estimate the pose using gyros and encoders when using megatag 2.

All of the math and code is mostly done within the Limelight itself via LimelightOS. You mainly just need to connect it to your robot and determine the protocol it sends to.

## License
This project is under the [MIT License](/LICENSE)
