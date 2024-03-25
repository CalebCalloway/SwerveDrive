package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    public class Robot extends SampleRobot {
        AHRS ahrs;
        RobotDrive myRobot;
        Joystick stick;

        public Robot() {
            myRobot = new RobotDrive(0, 1);
            myRobot.setExpiration(0.1);
            stick = new Joystick(0);
            try {
                /* Communicate w/navX-MXP via the MXP SPI Bus. */
                /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
                /*
                 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
                 * details.
                 */
                ahrs = new AHRS(SPI.Port.kMXP);
            } catch (RuntimeException ex) {
                DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
            }
        }

        /**
         * Drive left & right motors for 2 seconds then stop
         */
        public void autonomous() {
            myRobot.setSafetyEnabled(false);
            myRobot.drive(-0.5, 0.0); // drive forwards half speed
            Timer.delay(2.0); // for 2 seconds
            myRobot.drive(0.0, 0.0); // stop robot
        }

        /**
         * Runs the motors with arcade steering.
         */
        public void operatorControl() {
            myRobot.setSafetyEnabled(true);
            while (isOperatorControl() && isEnabled()) {
                if (stick.getRawButton(0)) {
                    ahrs.reset();
                }
                try {
                    /* Use the joystick X axis for lateral movement, */
                    /* Y axis for forward movement, and Z axis for rotation. */
                    /* Use navX-MXP yaw angle to define Field-centric transform */
                    myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(),
                            stick.getTwist(), ahrs.getAngle());
                } catch (RuntimeException ex) {
                    DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
                }
                Timer.delay(0.005); // wait for a motor update time
            }
        }

        /**
         * Runs during test mode
         */
        public void test() {
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
