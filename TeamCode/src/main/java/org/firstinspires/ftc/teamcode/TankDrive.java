package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.ftc.*;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.profiles.*;
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectories.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.trajectories.TurnConstraints;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.roadrunner.TrajectoryCommandBuilder;

import java.lang.Math;
import java.util.*;

@Config
public final class TankDrive extends Subsystem implements Drive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 0;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double ramseteZeta = 0.7; // in the range (0, 1)
        public double ramseteBBar = 2.0; // positive

        // turn controller gains
        public double turnGain = 0.0;
        public double turnVelGain = 0.0;
    }
    public static final TankDrive INSTANCE = new TankDrive();
    private TankDrive() {}

    public static Params PARAMS = new Params();

    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

    @NonNull
    @Override
    public TurnConstraints getDefaultTurnConstraints() {
        return defaultTurnConstraints;
    }

    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    new WheelVelConstraint(kinematics, PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));

    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    private final RamseteController ramseteController = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar);

    @NonNull
    @Override
    public RobotPosVelController getController() {
        return ramseteController;
    }

    public final FollowerParams followerParams = new FollowerParams(
            new ProfileParams(
                    0.25, 0.1, 1e-2
            ),
            defaultVelConstraint, defaultAccelConstraint
    );

    @NonNull
    @Override
    public FollowerParams getFollowerParams() {
        return followerParams;
    }

    public List<DcMotorEx> leftMotors, rightMotors;

    public LazyImu lazyImu;

    public VoltageSensor voltageSensor;

    public Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs;
        private Pose2d pose;

        private double lastLeftPos, lastRightPos;
        private boolean initialized;
        private ArrayDeque<Pose2d> poseHistory = new ArrayDeque<>();

        public DriveLocalizer(Pose2d pose) {
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    leftEncs.add(e);
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }

            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public ArrayList<Pose2d> getPoseHistory() {
            return new ArrayList<>(poseHistory);
        }

        @Override
        public PoseVelocity2d update() {
            Twist2dDual<Time> delta;

            List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
                leftReadings.add(p);
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
                rightReadings.add(p);
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                     new TankLocalizerInputsMessage(leftReadings, rightReadings));

            if (!initialized) {
                initialized = true;

                lastLeftPos = meanLeftPos;
                lastRightPos = meanRightPos;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);

            }

            Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.TankWheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            pose = pose.plus(twist.value());

            poseHistory.addFirst(pose);
            if (poseHistory.size() > 100) {
                poseHistory.removeLast();
            }

            return twist.velocity().value();
        }
    }

    public void initialize(HardwareMap hardwareMap) {
        assert hardwareMap != null;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse motor directions if needed
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer(Pose2d.zero);

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    @Override
    public void initialize() {
        initialize(OpModeData.hardwareMap);
    }

    @Override
    public Localizer getLocalizer() {
        return localizer;
    }

    @Override
    public void setDrivePowers(PoseVelocity2dDual<Time> powers) {
        TankKinematics.TankWheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(powers);

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    @Override
    public void setDrivePowers(PoseVelocity2d powers) {
        setDrivePowers(PoseVelocity2dDual.constant(powers, 1));
    }

    @Override
    public void setDrivePowersWithFF(PoseVelocity2dDual<Time> powers) {
        TankKinematics.TankWheelVelocities<Time> wheelVels = kinematics.inverse(powers);
        double voltage = voltageSensor.getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftPower = feedforward.compute(wheelVels.left) / voltage;
        double rightPower = feedforward.compute(wheelVels.right) / voltage;
        tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

        for (DcMotorEx m : leftMotors) {
            m.setPower(leftPower);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(rightPower);
        }
    }

    @Override
    public void setDrivePowersWithFF(PoseVelocity2d powers) {
        setDrivePowersWithFF(PoseVelocity2dDual.constant(powers, 1));
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));


        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryCommandBuilder commandBuilder(Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                turn -> new TurnCommand(turn, this),
                traj -> new FollowTrajectoryCommand(
                        new TimeFollower(traj, this),
                        this
                ),
                new TrajectoryBuilderParams(
                        1e-6,
                        followerParams.profileParams
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public TrajectoryCommandBuilder commandBuilder() {
        return commandBuilder(localizer.getPose());
    }

    @Override
    public TrajectoryBuilder trajectoryBuilder(Pose2d beginPose) {
        return new TrajectoryBuilder(
                new TrajectoryBuilderParams(
                        1e-6,
                        followerParams.profileParams
                ),
                beginPose, 0.0,
                defaultVelConstraint,
                defaultAccelConstraint
        );
    }
}
