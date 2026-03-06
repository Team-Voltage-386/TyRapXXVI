package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.*;
import java.io.IOException;
import lombok.Getter;
import org.json.simple.parser.ParseException;

public class AutoWrapper {

  @Getter protected String name;
  @Getter protected PathPlannerPath initialPath;
  @Getter protected boolean resetNav;
  @Getter protected Command autoCommand;

  public AutoWrapper(String name, String initialPathName, boolean resetNav, Command autoCommand)
      throws FileVersionException, IOException, ParseException {
    this.name = name;
    this.resetNav = resetNav;
    this.autoCommand = autoCommand;
    if (initialPathName != null) {
      this.initialPath = PathPlannerPath.fromPathFile(initialPathName);
    } else {
      this.initialPath = null;
    }
  }
}
