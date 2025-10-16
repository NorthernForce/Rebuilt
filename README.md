# Rebuilt

FRC Team 172 Northern Force's robot code for the 2025 season, built with WPILib and Java.

## About

This repository contains the robot code for **Team 172 - Northern Force**, a FIRST Robotics Competition team. The project uses the WPILib framework to control our competition robot, featuring a swerve drive system, AprilTag vision processing, and autonomous path planning.

## Prerequisites

Before you can build and deploy this project, you'll need:

- **WPILib 2025** - Download and install from [WPILib Releases](https://github.com/wpilibsuite/allwpilib/releases)
- **Java 17** - Included with WPILib installation
- **Git** - For version control
- **Gradle** - Included via Gradle wrapper (`gradlew`)

## Building the Project

To build the robot code:

```bash
./gradlew build
```

This will compile the code and run all tests. The build process creates a deployable JAR file that can be uploaded to the robot.

## Running the Code

### Simulating the Robot

To run the robot code in simulation mode:

```bash
./gradlew simulateJava
```

This launches the simulation GUI where you can test robot functionality without physical hardware.

### Deploying to RoboRIO

To deploy the code to the robot:

1. Connect to the robot's network (either via WiFi or USB)
2. Run the deploy command:

```bash
./gradlew deploy
```

This will build the code and upload it to the RoboRIO (Team 172's robot controller).

## Project Structure

- `src/main/java/frc/robot/` - Main robot code
  - `Robot.java` - Main robot class
  - `zippy/` - Configuration and container for the "Zippy" robot
    - `ZippyContainer.java` - Robot subsystems and command bindings
    - `ZippyConstants.java` - Robot-specific constants
    - `generated/` - Auto-generated swerve drive configuration
- `src/main/java/org/northernforce/util/` - Team utility classes
- `src/main/java/frc/robot/subsystems/` - Robot subsystems
  - `CommandSwerveDrivetrain.java` - Swerve drive subsystem
  - `apriltagvision/` - AprilTag vision processing
- `src/main/deploy/` - Files deployed to the robot

## Development Tools

### Code Formatting

This project uses Spotless for code formatting. Code is automatically formatted during compilation:

```bash
./gradlew spotlessApply
```

### Running Tests

To run the unit tests:

```bash
./gradlew test
```

## Additional Commands

- `./gradlew clean` - Clean build artifacts
- `./gradlew tasks` - List all available Gradle tasks
- `./gradlew discoverroborio` - Find the robot on the network

## License

This project is licensed under the WPILib BSD license. See [WPILib-License.md](WPILib-License.md) for details.

## Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [PhotonVision Documentation](https://docs.photonvision.org/)
