// Copyright 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.mcp;

import org.json.simple.JSONObject;

/**
 * Interface for MCP tools that can be registered dynamically
 *
 * <p>Implement this interface to create custom tools for the MCP server.
 */
public interface MCPTool {
  /** Returns the unique name of this tool */
  String getName();

  /** Returns a human-readable description of what this tool does */
  String getDescription();

  /** Returns the input schema (JSON Schema format) defining required/optional parameters */
  JSONObject getInputSchema();

  /**
   * Executes the tool with the provided arguments
   *
   * @param arguments The arguments passed from the AI agent
   * @return A JSONObject containing the result (should include "success" and relevant data)
   */
  JSONObject execute(JSONObject arguments);
}
