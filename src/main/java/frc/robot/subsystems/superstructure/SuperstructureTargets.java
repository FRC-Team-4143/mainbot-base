package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

public class SuperstructureTargets {

  public enum Target {
    // TODO: Update all the height to account for 0 being the new default position
    SAFETY(0.90041, Rotation2d.fromDegrees(-45)),
    CLIMB(0.90041, Rotation2d.fromDegrees(121.201)),
    CORAL_INTAKE(0.75, Rotation2d.fromDegrees(-117)),
    L4(2.109, Rotation2d.fromDegrees(-19)),
    L4_STAGING(Optional.empty(), Rotation2d.fromDegrees(-65)),
    L4_SAFETY(2.109, Rotation2d.fromDegrees(-19.599)),
    L3(1.323265, Rotation2d.fromDegrees(-5.05468)),
    L3_STAGING(Optional.empty(), Rotation2d.fromDegrees(-65)),
    L3_FAR(1.423500, Rotation2d.fromDegrees(-4.515)),
    L2(0.94895, Rotation2d.fromDegrees(-3.6035)),
    L2_STAGING(Optional.empty(),Rotation2d.fromDegrees(-65)),
    L2_FAR(1.0235, Rotation2d.fromDegrees(-4.515)),
    L1(0.824, Rotation2d.fromDegrees(-23.703)),
    L1_STAGING(Optional.empty(), Rotation2d.fromDegrees(0)),
    L1_FLICK(0.919, Rotation2d.fromDegrees(0)),
    ALGAE_STOW(0.90769, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_LOW(0.90848, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_HIGH(1.35857, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_PROCESSOR(0.7829, Rotation2d.fromDegrees(-40.1660)),
    BARGE(2.08, Rotation2d.fromDegrees(23.3789)),
    STATION(0.90041, Rotation2d.fromDegrees(135));

    public Optional<Double> elevator_height_ = Optional.empty();
    private double elevator_offset_ = 0;
    public Rotation2d arm_angle_ = Rotation2d.kZero;
    private Rotation2d arm_offset_ = Rotation2d.kZero;

    Target(Optional<Double> elevator_height, Rotation2d arm_angle) {
      elevator_height_ = elevator_height;
      arm_angle_ = arm_angle;
    }

    Target(double elevator_height, Rotation2d arm_angle) {
      elevator_height_ = Optional.of(elevator_height);
      arm_angle_ = arm_angle;
    }

    // Height Methods
    /**
     * Returns the current height including the active offset
     *
     * @return
     */
    public double getHeight() {
      if(elevator_height_.isEmpty()) {
        throw new IllegalStateException("This target does not have a defined elevator height");
      } else {
        return elevator_height_.get() + elevator_offset_;
      }
    }

    /**
     * Adjusts the height offset by the supplied increment
     *
     * @param offset
     */
    public void offsetHeight(double offset) {
      elevator_offset_ += offset;
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public double getHeightOffset() {
      return elevator_offset_;
    }

    /** Reset the height offset to 0 */
    public void resetHeightOffset() {
      elevator_offset_ = 0;
    }

    // Angle Methods
    /**
     * Returns the current angle including the active offset
     *
     * @return target angle
     */
    public Rotation2d getAngle() {
      return arm_angle_.rotateBy(arm_offset_);
    }

    /**
     * Adjusts the angle offset by the supplied increment
     *
     * @param offset
     */
    public void offsetAngle(Rotation2d offset) {
      arm_offset_ = arm_offset_.rotateBy(offset);
    }

    /**
     * Returns the current stored angle offset
     *
     * @return
     */
    public Rotation2d getAngleOffset() {
      return arm_offset_;
    }

    /** Resets the angle offset to 0 */
    public void resetAngleOffset() {
      arm_offset_ = new Rotation2d();
    }

    /** Resets both height and angle offsets */
    public void resetOffsets() {
      resetAngleOffset();
      resetHeightOffset();
    }
  }
}
