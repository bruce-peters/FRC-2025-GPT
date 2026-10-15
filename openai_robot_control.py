"""
OpenAI-powered robot control via MCP
Uses GPT-4o-mini to control the FRC robot through MCP tools
"""

import os
import sys
import requests
import json
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configuration
ROBOT_URL = os.environ.get("ROBOT_URL", "http://localhost:3000")
OPENAI_API_KEY = os.environ.get("OPENAI_API_KEY")

if not OPENAI_API_KEY:
    print("Error: OPENAI_API_KEY is not set.")
    print("Copy .env.example to .env and put your OpenAI API key in it (or export OPENAI_API_KEY).")
    sys.exit(1)

# Initialize OpenAI client
client = OpenAI(api_key=OPENAI_API_KEY)

def get_robot_tools():
    """Get available tools from the robot's MCP server"""
    try:
        url = f"{ROBOT_URL}/mcp/v1/tools/list"
        response = requests.get(url, timeout=5)
        response.raise_for_status()
        tools_data = response.json()
        
        # Convert MCP tools to OpenAI function format
        openai_tools = []
        for tool in tools_data.get("tools", []):
            openai_tool = {
                "type": "function",
                "function": {
                    "name": tool["name"],
                    "description": tool["description"],
                    "parameters": tool["inputSchema"]
                }
            }
            openai_tools.append(openai_tool)
            print(f"  - {tool['name']}")
        
        return openai_tools
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to robot at {ROBOT_URL}: {e}")
        return []
    except Exception as e:
        print(f"Error getting tools: {e}")
        return []

def call_robot_tool(tool_name, arguments):
    """Call a tool on the robot"""
    try:
        url = f"{ROBOT_URL}/mcp/v1/tools/call"
        print(f"Calling robot tool at: {url}")
        # Use longer timeout for ScanForCoral which can take time
        timeout = 60 if tool_name == "ScanForCoral" else 10
        response = requests.post(
            url,
            json={"name": tool_name, "arguments": arguments},
            timeout=timeout
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        return {"error": f"Request error: {str(e)}", "url": url}
    except Exception as e:
        return {"error": str(e)}

def get_robot_status():
    """Get current robot status from MCP server"""
    try:
        result = call_robot_tool("GetRobotStatus", {})
        if "error" in result:
            return None
        return result
    except Exception as e:
        print(f"Failed to get robot status: {e}")
        return None

def chat_with_robot(user_message, conversation_history=None):
    """Send a message to the AI and execute any tool calls"""
    if conversation_history is None:
        conversation_history = []
    
    # Add user message to history
    conversation_history.append({"role": "user", "content": user_message})
    
    # Get available tools
    tools = get_robot_tools()
    
    if not tools:
        print("Warning: No tools available from robot")
    
    # Create system message explaining the robot's capabilities
    system_message = {
        "role": "system",
        "content": """You are an AI assistant that can control an FRC robot. 

IMPORTANT: First, determine if the user is asking you to control the robot or just asking a general question:
- If it's a GENERAL question (like "what's your favorite color?", "tell me a joke", etc.) → Just answer the question normally WITHOUT using any tools
- If it's a ROBOT CONTROL request (like "collect coral", "drive to position", "scan for objects") → Use the available tools to complete the task

Before calling any robot tool, briefly explain your reasoning in 2-3 sentences MAX. Be concise - just state what you observe and what action you'll take.

ROBOT STATUS: The current robot status (position, coral detection, reef distances) is automatically provided before every decision when working on robot tasks. You don't need to call GetRobotStatus - the info is already in the context!

AVAILABLE TOOLS (only use for robot control tasks):
- GetRobotStatus: Get current position, superstructure state, detected objects, and reef distances (usually not needed - status is auto-provided)
- ScanForCoral: Rotates 360° while scanning for coral. Returns true if coral found and its position.
- IntakeClosestObject: Drives to the closest detected coral and picks it up automatically
- DriveToPoint: Move to x,y,theta coordinates (field-absolute)
- MoveRelative: Move forward/left relative to robot's current orientation
- RotateRobot: Rotate by a specified number of degrees
- SetSuperstructureState: Set elevator/arm state (STOW, L2_ALGAE, L3_ALGAE, etc.)
- StopRobot: Emergency stop all movement

FIELD INFORMATION:
- Field size: 16.54m x 8.21m
- Coral locations: Typically scattered on edges (y<2m or y>6m) and corners
- Reefs: Blue reef at x≈4.5, Red reef at x≈13, both at y≈4
- Avoid getting too close to reefs unless scoring

CORAL COLLECTION TIPS (if that's the user's goal):
- When coral_detected=true in status → use IntakeClosestObject (it drives AND intakes)
- When no coral visible → use ScanForCoral to look around
- If ScanForCoral finds nothing → DriveToPoint to explore new areas (edges/corners)
- Move at least 5+ meters between exploration spots
- IntakeClosestObject handles both driving to and picking up coral

GENERAL TIPS:
- Keep reasoning brief (2-3 sentences before each action)
- Use the robot status to make informed decisions
- If a command fails or robot gets stuck, try a different approach
- Keep working on robot tasks until complete or you run out of iterations"""
    }
    
    # Agentic loop - keep calling tools until the AI is done
    max_iterations = 100
    iteration = 0
    
    try:
        while iteration < max_iterations:
            iteration += 1
            
            # Get current robot status and inject it as context
            robot_status = get_robot_status()
            status_context = ""
            if robot_status:
                status_context = f"\n\nCURRENT ROBOT STATUS:\n{json.dumps(robot_status, indent=2)}"
            
            # Add status context to the most recent message for the AI
            messages_with_status = [system_message] + conversation_history.copy()
            if messages_with_status and messages_with_status[-1]["role"] == "user":
                messages_with_status[-1] = {
                    "role": "user",
                    "content": messages_with_status[-1]["content"] + status_context
                }
            elif status_context:
                # If last message isn't user, append as new user message
                messages_with_status.append({"role": "user", "content": status_context})
            
            # Call OpenAI API with streaming enabled
            stream = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages_with_status,
                tools=tools if tools else None,
                tool_choice="auto" if tools else None,
                stream=True
            )
            
            # Collect streamed response
            message_content = ""
            tool_calls_data = []
            current_tool_call = None
            
            print(f"\n[AI thinking (iteration {iteration})]: ", end="", flush=True)
            
            for chunk in stream:
                delta = chunk.choices[0].delta
                
                # Stream text content (reasoning)
                if delta.content:
                    print(delta.content, end="", flush=True)
                    message_content += delta.content
                
                # Collect tool calls
                if delta.tool_calls:
                    for tc_delta in delta.tool_calls:
                        if tc_delta.index is not None:
                            # New tool call starting
                            while len(tool_calls_data) <= tc_delta.index:
                                tool_calls_data.append({
                                    "id": "",
                                    "type": "function",
                                    "function": {"name": "", "arguments": ""}
                                })
                            current_tool_call = tool_calls_data[tc_delta.index]
                        
                        if current_tool_call is not None:
                            if tc_delta.id:
                                current_tool_call["id"] = tc_delta.id
                            if tc_delta.function:
                                if tc_delta.function.name:
                                    current_tool_call["function"]["name"] = tc_delta.function.name
                                if tc_delta.function.arguments:
                                    current_tool_call["function"]["arguments"] += tc_delta.function.arguments
            
            print()  # New line after streaming
            
            # If AI wants to use tools
            if tool_calls_data:
                # Add assistant message with tool calls to history
                conversation_history.append({
                    "role": "assistant",
                    "content": message_content if message_content else None,
                    "tool_calls": tool_calls_data
                })
                
                # Execute each tool call
                for tool_call in tool_calls_data:
                    tool_name = tool_call["function"]["name"]
                    arguments = json.loads(tool_call["function"]["arguments"])
                    
                    print(f"\nExecuting: {tool_name}({json.dumps(arguments)})")
                    
                    # Call the robot tool
                    result = call_robot_tool(tool_name, arguments)
                    print(f"Result: {json.dumps(result, indent=2)}")
                    
                    # Add tool result to conversation
                    conversation_history.append({
                        "role": "tool",
                        "tool_call_id": tool_call["id"],
                        "content": json.dumps(result)
                    })
                
                # Continue the loop to let AI decide if it needs more tools
                
            else:
                # No tool calls - just reasoning or final answer
                if message_content:
                    # Check if AI is indicating it will take an action but didn't make a tool call
                    action_indicators = ["let's", "i'll", "i will", "going to", "will now", "will execute"]
                    is_planning_action = any(indicator in message_content.lower() for indicator in action_indicators)
                    
                    if is_planning_action and iteration < max_iterations:
                        # AI said it would do something but didn't make a tool call
                        # Add the message and prompt it to actually make the call
                        conversation_history.append({"role": "assistant", "content": message_content})
                        conversation_history.append({
                            "role": "user", 
                            "content": "Please execute the tool call now (don't just describe what you'll do)."
                        })
                        print("[Prompting AI to execute tool...]")
                        continue
                    else:
                        # Final answer or non-action message
                        conversation_history.append({"role": "assistant", "content": message_content})
                        return message_content, conversation_history
                else:
                    # Empty response, continue
                    continue
                    continue
        
        # If we hit max iterations, return what we have
        final_message = "Reached maximum iterations. Task may be incomplete."
        conversation_history.append({"role": "assistant", "content": final_message})
        return final_message, conversation_history
            
    except Exception as e:
        error_msg = f"Error communicating with OpenAI: {e}"
        print(error_msg)
        return error_msg, conversation_history

def main():
    """Interactive chat loop"""
    print("=" * 60)
    print("FRC Robot Control with OpenAI GPT-4o-mini")
    print("=" * 60)
    print("\nMake sure your robot simulation is running!")
    print("Type 'quit' or 'exit' to stop\n")
    
    conversation_history = []
    
    while True:
        try:
            user_input = input("\n[You]: ").strip()
            
            if user_input.lower() in ['quit', 'exit', 'q']:
                print("\nGoodbye!")
                break
            
            if not user_input:
                continue
            
            print("\nAI is thinking...")
            response, conversation_history = chat_with_robot(user_input, conversation_history)
            
            print(f"\n[AI]: {response}")
            
        except KeyboardInterrupt:
            print("\nExiting")
            break
        except Exception as e:
            print(f"\nError: {e}")

if __name__ == "__main__":
    # Check if robot is accessible
    try:
        response = requests.get(f"{ROBOT_URL}/mcp/v1/tools/list", timeout=2)
        print("Robot MCP server is accessible")
    except:
        print(f"Warning: Cannot connect to robot MCP server at {ROBOT_URL}")
        print("   Make sure your robot simulation is running!")
        print()
    
    main()
