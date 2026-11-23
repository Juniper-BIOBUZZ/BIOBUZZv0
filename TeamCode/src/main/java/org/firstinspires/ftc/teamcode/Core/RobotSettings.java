package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

//odo offsets: (horizontal: 150.781mm, vertical: 43.563mm),
//center of robot to center line: 187.176 + 220
//center of robot to the wall: 260.35474
//middle of frame to middle of cell: 323.85mm
//pivot to middle of cell: 509.501mm

// inches, 0,0 = field center, +X away from the audience, +Y toward red, heading 0 = facing the back wall
@Config
public class RobotSettings {
    public static double HIVE_Y = 12.75;
    public static double CELL_X = 12.6;

    public static double ROBOT_HALF_LENGTH = 10.25;
    public static double START_Y = 24;
    public static double START_HEADING = 180;

    public enum Alliance {
        RED(1, Cell.AUDIENCE_SIDE),
        BLUE(-1, Cell.FAR_SIDE);

        private final int side;
        private final Cell startUpCell;

        Alliance(int side, Cell startUpCell) {
            this.side = side;
            this.startUpCell = startUpCell;
        }

        public Pose2D getCell(Cell cell) {
            double x = cell == Cell.AUDIENCE_SIDE ? -CELL_X : CELL_X;
            return new Pose2D(DistanceUnit.INCH, x, side * HIVE_Y, AngleUnit.DEGREES, 0);
        }

        public Cell startUpCell() {
            return startUpCell;
        }

        public Pose2D getGoalPos() {
            return getCell(startUpCell);
        }
    }

    public enum Cell {
        AUDIENCE_SIDE,
        FAR_SIDE;

        public Cell other() {
            return this == AUDIENCE_SIDE ? FAR_SIDE : AUDIENCE_SIDE;
        }
    }

    public enum DistanceMethod {
        LIMELIGHT_ONLY,
        PINPOINT_ONLY,
        LIMELIGHT_AND_PINPOINT
    }

    public enum StartPos {
        RED_AUDIENCE_WALL(Alliance.RED),
        BLUE_BACK_WALL(Alliance.BLUE);

        private final Alliance alliance;

        StartPos(Alliance alliance) {
            this.alliance = alliance;
        }

        public Alliance getAlliance() {
            return alliance;
        }

        public Pose2D getPose2D() {
            double x = 72 - ROBOT_HALF_LENGTH;
            if (alliance == Alliance.RED) {
                return new Pose2D(DistanceUnit.INCH, -x, START_Y, AngleUnit.DEGREES, START_HEADING);
            }
            return new Pose2D(DistanceUnit.INCH, x, -START_Y, AngleUnit.DEGREES, AngleUnit.normalizeDegrees(START_HEADING + 180));
        }

        public static StartPos forAlliance(Alliance alliance) {
            for (StartPos pos : values()) {
                if (pos.alliance == alliance) return pos;
            }
            return RED_AUDIENCE_WALL;
        }
    }

    public Alliance alliance;
    public DistanceMethod distanceMethod;
    public StartPos startPosState;

    private static final String FILENAME = "RobotSettings.json";

    public RobotSettings() {
        alliance = Alliance.RED;
        distanceMethod = DistanceMethod.PINPOINT_ONLY;
        startPosState = StartPos.RED_AUDIENCE_WALL;
    }

    public void save() {
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        try (FileWriter writer = new FileWriter(file)) {
            gson.toJson(this, writer);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static RobotSettings load() {
        RobotSettings defaults = new RobotSettings();
        File file = AppUtil.getInstance().getSettingsFile(FILENAME);
        try (FileReader reader = new FileReader(file)) {
            RobotSettings loaded = new Gson().fromJson(reader, RobotSettings.class);
            if (loaded == null) return defaults;
            if (loaded.alliance == null) loaded.alliance = defaults.alliance;
            if (loaded.distanceMethod == null) loaded.distanceMethod = defaults.distanceMethod;
            if (loaded.startPosState == null || loaded.startPosState.getAlliance() != loaded.alliance) {
                loaded.startPosState = StartPos.forAlliance(loaded.alliance);
            }
            return loaded;
        } catch (Exception e) {
            return defaults;
        }
    }
}
