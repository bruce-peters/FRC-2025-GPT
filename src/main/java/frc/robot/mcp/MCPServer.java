// Copyright 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.

package frc.robot.mcp;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import frc.robot.RobotContainer;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.Executors;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * MCP (Model Context Protocol) Server for FRC Robot Exposes robot capabilities as tools that can be
 * called by AI agents
 */
public class MCPServer {
  private static final int DEFAULT_PORT = 3000;
  private HttpServer server;
  private RobotContainer robotContainer;
  private MCPTools tools;

  public MCPServer(RobotContainer robotContainer) {
    this(robotContainer, DEFAULT_PORT);
  }

  public MCPServer(RobotContainer robotContainer, int port) {
    this.robotContainer = robotContainer;
    this.tools = new MCPTools(robotContainer);

    try {
      server = HttpServer.create(new InetSocketAddress(port), 0);
      setupEndpoints();
      server.setExecutor(Executors.newFixedThreadPool(4));
      System.out.println("MCP Server initialized on port " + port);
    } catch (IOException e) {
      System.err.println("Failed to start MCP server: " + e.getMessage());
    }
  }

  /**
   * Get the tools manager to register custom tools
   *
   * @return The MCPTools instance for tool registration
   */
  public MCPTools getTools() {
    return tools;
  }

  public void start() {
    if (server != null) {
      server.start();
      System.out.println("MCP Server started. Listening for AI agent requests...");
    }
  }

  public void stop() {
    if (server != null) {
      server.stop(0);
      System.out.println("MCP Server stopped.");
    }
  }

  private void setupEndpoints() {
    // MCP protocol endpoints
    server.createContext("/mcp/v1/initialize", this::handleInitialize);
    server.createContext("/mcp/v1/tools/list", this::handleToolsList);
    server.createContext("/mcp/v1/tools/call", this::handleToolCall);

    server.createContext("/mcp/v1/prompts/list", this::handlePromptsList);

    // Health check
    server.createContext("/health", this::handleHealth);
  }

  private void handleInitialize(HttpExchange exchange) throws IOException {
    if (!"POST".equals(exchange.getRequestMethod())) {
      sendResponse(exchange, 405, "{\"error\": \"Method not allowed\"}");
      return;
    }

    JSONObject response = new JSONObject();
    response.put("protocolVersion", "2024-11-05");
    response.put("serverInfo", createServerInfo());
    response.put("capabilities", createCapabilities());

    sendResponse(exchange, 200, response.toJSONString());
  }

  private void handleToolsList(HttpExchange exchange) throws IOException {
    if (!"GET".equals(exchange.getRequestMethod())) {
      sendResponse(exchange, 405, "{\"error\": \"Method not allowed\"}");
      return;
    }

    JSONObject response = new JSONObject();
    response.put("tools", tools.getToolsList());

    sendResponse(exchange, 200, response.toJSONString());
  }

  private void handleToolCall(HttpExchange exchange) throws IOException {
    if (!"POST".equals(exchange.getRequestMethod())) {
      sendResponse(exchange, 405, "{\"error\": \"Method not allowed\"}");
      return;
    }

    try {
      String requestBody =
          new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
      JSONParser parser = new JSONParser();
      JSONObject request = (JSONObject) parser.parse(requestBody);

      String toolName = (String) request.get("name");
      JSONObject arguments = (JSONObject) request.get("arguments");

      JSONObject result = tools.callTool(toolName, arguments);

      JSONObject response = new JSONObject();
      response.put("content", result);

      sendResponse(exchange, 200, response.toJSONString());
    } catch (Exception e) {
      JSONObject error = new JSONObject();
      error.put("error", e.getMessage());
      sendResponse(exchange, 400, error.toJSONString());
    }
  }

  private void handlePromptsList(HttpExchange exchange) throws IOException {
    if (!"GET".equals(exchange.getRequestMethod())) {
      sendResponse(exchange, 405, "{\"error\": \"Method not allowed\"}");
      return;
    }

    JSONObject response = new JSONObject();
    response.put("prompts", new JSONArray());

    sendResponse(exchange, 200, response.toJSONString());
  }

  private void handleHealth(HttpExchange exchange) throws IOException {
    JSONObject response = new JSONObject();
    response.put("status", "healthy");
    response.put("timestamp", System.currentTimeMillis());

    sendResponse(exchange, 200, response.toJSONString());
  }

  private JSONObject createServerInfo() {
    JSONObject info = new JSONObject();
    info.put("name", "FRC-Robot-MCP-Server");
    info.put("version", "1.0.0");
    return info;
  }

  private JSONObject createCapabilities() {
    JSONObject capabilities = new JSONObject();
    capabilities.put("tools", true);
    capabilities.put("prompts", false);
    return capabilities;
  }

  private void sendResponse(HttpExchange exchange, int statusCode, String response)
      throws IOException {
    byte[] responseBytes = response.getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().add("Content-Type", "application/json");
    exchange.getResponseHeaders().add("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(statusCode, responseBytes.length);

    try (OutputStream os = exchange.getResponseBody()) {
      os.write(responseBytes);
    }
  }
}
