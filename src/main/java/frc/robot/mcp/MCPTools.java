package frc.robot.mcp;

import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

/** Manages MCP tools for robot operations with dynamic registration support */
public class MCPTools {
  private RobotContainer robotContainer;
  private Map<String, MCPTool> registeredTools;

  public MCPTools(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.registeredTools = new HashMap<>();
  }

  /**
   * Register a custom tool dynamically
   *
   * @param tool The tool to register
   */
  public void registerTool(MCPTool tool) {
    registeredTools.put(tool.getName(), tool);
    System.out.println("MCP: Registered tool '" + tool.getName() + "'");
  }

  /**
   * Register a tool using a builder pattern for simpler inline definitions
   *
   * @param name Tool name
   * @param description Tool description
   * @param inputSchema JSON Schema for parameters
   * @param executor Function to execute the tool
   */
  public void registerTool(
      String name, String description, JSONObject inputSchema, ToolExecutor executor) {
    registerTool(new SimpleMCPTool(name, description, inputSchema, executor));
  }

  /** Returns list of available tools with their schemas */
  public JSONArray getToolsList() {
    JSONArray tools = new JSONArray();

    // Add all registered tools
    for (MCPTool tool : registeredTools.values()) {
      JSONObject toolDef = new JSONObject();
      toolDef.put("name", tool.getName());
      toolDef.put("description", tool.getDescription());
      toolDef.put("inputSchema", tool.getInputSchema());
      tools.add(toolDef);
    }

    return tools;
  }

  /** Executes a tool by name with given arguments */
  public JSONObject callTool(String toolName, JSONObject arguments) {
    try {
      MCPTool tool = registeredTools.get(toolName);
      if (tool == null) {
        JSONObject error = new JSONObject();
        error.put("error", "Unknown tool: " + toolName);
        return error;
      }
      return tool.execute(arguments);
    } catch (Exception e) {
      JSONObject error = new JSONObject();
      error.put("error", e.getMessage());
      error.put("stackTrace", e.toString());
      return error;
    }
  }

  /** Functional interface for tool execution */
  @FunctionalInterface
  public interface ToolExecutor {
    JSONObject execute(JSONObject arguments) throws Exception;
  }

  // ==================== Helper Methods for Schema Creation ====================

  /** Create a parameter schema for a number parameter */
  public static JSONObject createNumberParam(String description) {
    JSONObject param = new JSONObject();
    param.put("type", "number");
    param.put("description", description);
    return param;
  }

  /** Create a parameter schema for a string parameter */
  public static JSONObject createStringParam(String description) {
    JSONObject param = new JSONObject();
    param.put("type", "string");
    param.put("description", description);
    return param;
  }

  /** Create an input schema with specified properties and required fields */
  public static JSONObject createInputSchema(JSONObject properties, String... requiredFields) {
    JSONObject schema = new JSONObject();
    schema.put("type", "object");
    schema.put("properties", properties);

    if (requiredFields.length > 0) {
      JSONArray required = new JSONArray();
      for (String field : requiredFields) {
        required.add(field);
      }
      schema.put("required", required);
    }

    return schema;
  }

  public static JSONObject successResponse() {
    JSONObject response = new JSONObject();
    response.put("success", true);
    return response;
  }

  // ==================== Legacy Tool Definition Methods (for reference) ====================
  // These are kept as examples but are no longer used directlys
  class SimpleMCPTool implements MCPTool {
    private final String name;
    private final String description;
    private final JSONObject inputSchema;
    private final ToolExecutor executor;

    public SimpleMCPTool(
        String name, String description, JSONObject inputSchema, ToolExecutor executor) {
      this.name = name;
      this.description = description;
      this.inputSchema = inputSchema;
      this.executor = executor;
    }

    @Override
    public String getName() {
      return name;
    }

    @Override
    public String getDescription() {
      return description;
    }

    @Override
    public JSONObject getInputSchema() {
      return inputSchema;
    }

    @Override
    public JSONObject execute(JSONObject arguments) {
      try {
        return executor.execute(arguments);
      } catch (Exception e) {
        JSONObject error = new JSONObject();
        error.put("success", false);
        error.put("error", e.getMessage());
        return error;
      }
    }
  }
}
