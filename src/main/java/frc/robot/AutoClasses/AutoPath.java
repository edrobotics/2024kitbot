package frc.robot.AutoClasses;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Functions;

public class AutoPath {
  public ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
  public double speed;
  public int length = 0;

  public AutoPath(String autoName) {
    Path filePath = Paths.get(Filesystem.getDeployDirectory().toString(), "Autos", autoName+".json");
    String json;
    try {
      json = Files.readString(filePath);
      ObjectMapper objectMapper = new ObjectMapper();
      JsonNode rootNode;
      rootNode = objectMapper.readTree(json);

      speed = rootNode.path("speed").asDouble();
      JsonNode waypointsNode = rootNode.path("waypoints");
      if(waypointsNode.isArray()) {
        for(JsonNode thisWaypointNode : waypointsNode) {
          Waypoint thisWaypoint = new Waypoint(thisWaypointNode.path("x").asDouble(), thisWaypointNode.path("y").asDouble(), thisWaypointNode.path("speed").asDouble());
          waypoints.add(thisWaypoint);
          length++;
        }
      }
    }
    catch(Exception e) {
      Functions.printInTerminal("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Functions.printInTerminal("Error loading auto file: " + filePath);
      Functions.printInTerminal("Deploy directory: " + Filesystem.getDeployDirectory());
      e.printStackTrace();
      Functions.printInTerminal("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
  }
}
