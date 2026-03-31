// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.libyear;

import com.marswars.geometry.AllianceFlipUtil.SymmetryType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>NOTE: All constants are defined relative to the field coordinate system, and from the
 * perspective of the blue alliance station
 */
public class FieldConstants {

    public static final SymmetryType FIELD_SYMMETRY_TYPE = SymmetryType.DIAGONAL;

    // AprilTag related constants
    public static final int APRIL_TAG_COUNT =
            AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
    public static final double APRIL_TAG_WIDTH = Units.inchesToMeters(6.5);
    public static final AprilTagLayoutType DEFAULT_APRIL_TAG_TYPE = AprilTagLayoutType.OFFICIAL;

    // Field dimensions
    public static final double FIELD_LENGTH =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double FIELD_WIDTH =
            AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();
    public static final Translation2d FIELD_CENTER =
            new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2);

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }
}
