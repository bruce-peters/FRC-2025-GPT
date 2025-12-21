"""
Simple MCP Client for FRC Robot
Demonstrates how to connect to and interact with the robot's MCP server
"""

import requests
import json
import time
import sys

class RobotMCPClient:
    def __init__(self, robot_ip="localhost", port=3000):
        self.base_url = f"http://{robot_ip}:{port}"
        self.initialized = False
        
    def check_health(self):
        """Check if the server is running"""
        try:
            response = requests.get(f"{self.base_url}/health", timeout=2)
            if response.status_code == 200:
                data = response.json()
                print(f"✓ Server is healthy: {data}")
                return True
        except requests.exceptions.RequestException as e:
            print(f"✗ Server health check failed: {e}")
            return False
    
    def initialize(self):
        """Initialize the MCP connection"""
        try:
            response = requests.post(f"{self.base_url}/mcp/v1/initialize", timeout=5)
            if response.status_code == 200:
                data = response.json()
                print(f"✓ MCP connection initialized")
                print(f"  Protocol version: {data.get('protocolVersion')}")
                print(f"  Server: {data.get('serverInfo', {}).get('name')}")
                self.initialized = True
                return True
        except requests.exceptions.RequestException as e:
            print(f"✗ Initialization failed: {e}")
            return False
    
    def list_tools(self):
        """List all available tools"""
        try:
            response = requests.get(f"{self.base_url}/mcp/v1/tools/list", timeout=5)
            if response.status_code == 200:
                data = response.json()
                tools = data.get('tools', [])
                print(f"\n✓ Available tools ({len(tools)}):")
                for tool in tools:
                    name = tool.get('name')
                    description = tool.get('description')
                    print(f"\n  • {name}")
                    print(f"    {description}")
                    schema = tool.get('inputSchema', {})
                    properties = schema.get('properties', {})
                    if properties:
                        print(f"    Parameters:")
                        for param_name, param_info in properties.items():
                            param_type = param_info.get('type', 'unknown')
                            param_desc = param_info.get('description', '')
                            required = param_name in schema.get('required', [])
                            req_marker = "*" if required else ""
                            print(f"      - {param_name}{req_marker} ({param_type}): {param_desc}")
                return tools
        except requests.exceptions.RequestException as e:
            print(f"✗ Failed to list tools: {e}")
            return []
    
    def call_tool(self, tool_name, arguments):
        """Call a specific tool with arguments"""
        try:
            payload = {
                "name": tool_name,
                "arguments": arguments
            }
            response = requests.post(
                f"{self.base_url}/mcp/v1/tools/call",
                json=payload,
                timeout=10
            )
            if response.status_code == 200:
                data = response.json()
                print(f"✓ Tool '{tool_name}' executed successfully")
                return data.get('content')
            else:
                print(f"✗ Tool call failed with status {response.status_code}")
                print(f"  Response: {response.text}")
                return None
        except requests.exceptions.RequestException as e:
            print(f"✗ Tool call failed: {e}")
            return None
    
    def call_any_tool(self, tool_name, **kwargs):
        """Generic method to call any tool with keyword arguments
        
        Example:
            client.call_any_tool("get_robot_pose")
            client.call_any_tool("drive_robot", xSpeed=0.5, ySpeed=0.0, rotSpeed=0.0, duration=1.0)
        """
        print(f"\n→ Calling tool '{tool_name}'...")
        if kwargs:
            print(f"  Arguments: {json.dumps(kwargs, indent=2)}")
        
        result = self.call_tool(tool_name, kwargs)
        if result:
            print(f"  Result: {json.dumps(result, indent=2)}")
        return result

def demo_basic_connection():
    """Demonstrate basic connection and tool listing"""
    print("=" * 60)
    print("FRC Robot MCP Client - Basic Connection Demo")
    print("=" * 60)
    
    # Connect to robot
    client = RobotMCPClient()
    
    # Check health
    print("\n1. Checking server health...")
    if not client.check_health():
        print("\nCannot connect to robot. Is it powered on and deployed?")
        return
    
    # Initialize
    print("\n2. Initializing MCP connection...")
    if not client.initialize():
        return
    
    # List available tools
    print("\n3. Listing available tools...")
    client.list_tools()

def demo_robot_control():
    """Demonstrate basic robot control using dynamic tools"""
    print("\n" + "=" * 60)
    print("FRC Robot MCP Client - Robot Control Demo")
    print("=" * 60)
    
    client = RobotMCPClient()
    
    # Quick connect
    if not client.check_health() or not client.initialize():
        print("\nCannot connect to robot.")
        return
    
    # List tools first
    print("\nFetching available tools...")
    tools = client.list_tools()
    
    print("\n\nDemonstrating tool calls...")
    
    try:
        # Example tool calls (adapt based on what's registered)
        print("\n1. Calling 'get_robot_pose' (if available)...")
        client.call_any_tool("get_robot_pose")
        
        print("\n2. Calling 'get_subsystem_status' (if available)...")
        client.call_any_tool("get_subsystem_status")
        
        # Drive example
        print("\n⚠️  About to call 'drive_robot' tool...")
        confirmation = input("Continue? (y/n): ")
        if confirmation.lower() == 'y':
            client.call_any_tool("drive_robot", xSpeed=0.5, ySpeed=0.0, rotSpeed=0.0, duration=1.0)
            time.sleep(1.5)
            
            # Stop
            client.call_any_tool("stop_all")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted! Stopping robot...")
        client.call_any_tool("stop_all")

def interactive_mode():
    """Interactive command-line interface with dynamic tool support"""
    print("\n" + "=" * 60)
    print("FRC Robot MCP Client - Interactive Mode")
    print("=" * 60)
    
    client = RobotMCPClient()
    
    if not client.check_health() or not client.initialize():
        print("\nCannot connect to robot.")
        return
    
    # Fetch available tools
    print("\nFetching available tools...")
    available_tools = client.list_tools()
    
    print("\nConnected! Type 'help' for commands, 'quit' to exit.")
    
    while True:
        try:
            command = input("\n> ").strip()
            
            if command.lower() in ['quit', 'exit']:
                break
            elif command.lower() == 'help':
                print("\nAvailable commands:")
                print("  tools      - List available tools")
                print("  call       - Call a tool by name")
                print("  quit/exit  - Exit interactive mode")
            elif command.lower() == 'tools':
                client.list_tools()
            elif command.lower() == 'call':
                tool_name = input("  Tool name: ").strip()
                
                # Find tool schema
                tool_schema = None
                if available_tools:
                    for t in available_tools:
                        if t.get('name') == tool_name:
                            tool_schema = t.get('inputSchema', {})
                            break
                
                # Collect parameters
                args = {}
                if tool_schema:
                    properties = tool_schema.get('properties', {})
                    required = tool_schema.get('required', [])
                    
                    if properties:
                        print(f"\n  Parameters for '{tool_name}':")
                    
                    for param_name, param_info in properties.items():
                        param_type = param_info.get('type', 'string')
                        param_desc = param_info.get('description', '')
                        is_required = param_name in required
                        
                        # Show description if available
                        if param_desc:
                            print(f"    {param_desc}")
                        
                        prompt = f"  {param_name} ({'required' if is_required else 'optional'}, {param_type}): "
                        value_str = input(prompt).strip()
                        
                        # Check if required parameter is missing
                        if is_required and not value_str:
                            print(f"    Error: '{param_name}' is required but no value provided.")
                            continue
                        
                        # Only add parameter if value was provided
                        if value_str:
                            # Convert to appropriate type
                            try:
                                if param_type == 'number':
                                    args[param_name] = float(value_str)
                                elif param_type == 'integer':
                                    args[param_name] = int(value_str)
                                elif param_type == 'boolean':
                                    args[param_name] = value_str.lower() in ['true', 'yes', '1']
                                else:
                                    args[param_name] = value_str
                            except ValueError:
                                print(f"    Warning: Could not convert '{value_str}' to {param_type}, using as string")
                                args[param_name] = value_str
                
                # Call the tool
                client.call_any_tool(tool_name, **args)
            else:
                print(f"Unknown command: {command}. Type 'help' for commands.")
                
        except KeyboardInterrupt:
            print("\n\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    print("""
╔══════════════════════════════════════════════════════════╗
║          FRC Robot MCP Client                            ║
║          Connect and control your robot via AI           ║
╚══════════════════════════════════════════════════════════╝
    """)
    
    print("Select a mode:")
    print("  1. Basic connection demo")
    print("  2. Robot control demo")
    print("  3. Interactive mode")
    print("  4. Exit")
    
    choice = input("\nChoice (1-4): ").strip()
    
    if choice == '1':
        demo_basic_connection()
    elif choice == '2':
        demo_robot_control()
    elif choice == '3':
        interactive_mode()
    elif choice == '4':
        print("Goodbye!")
    else:
        print("Invalid choice.")
