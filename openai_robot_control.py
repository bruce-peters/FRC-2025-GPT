"""
OpenAI-powered robot control via MCP
Uses GPT-4o-mini to control the FRC robot through MCP tools
"""

import os
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
    raise ValueError("OPENAI_API_KEY not found in .env file. Please create a .env file with your API key.")

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
        response = requests.post(
            url,
            json={"name": tool_name, "arguments": arguments},
            timeout=10
        )
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        return {"error": f"Request error: {str(e)}", "url": url}
    except Exception as e:
        return {"error": str(e)}

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
        "content": """You are controlling an FRC robot. Your ONLY goal is to collect as much coral as possible. KEEP WORKING until you run out of iterations.

TOOLS:
- GetRobotStatus: Shows position, detected coral, reef distances
- ScanForCoral: Rotates 360° while scanning for coral. Returns true if coral found.
- DriveToPoint: Move to x,y,theta coordinates
- IntakeClosestObject: Pick up detected coral
- StopRobot: DO NOT USE THIS - keep exploring!

FIELD: 16.54m x 8.21m
- Coral locations: Scattered on edges (y<2m or y>6m) and corners
- Reefs at x≈4.5 and x≈13, y≈4 (AVOID!)

EXACT STRATEGY TO FOLLOW:
1. GetRobotStatus - check if coral is already detected
2. IF coral detected → IntakeClosestObject immediately, then go to step 1
3. IF NO coral detected → ScanForCoral (this rotates 360° looking for coral)
4. IF ScanForCoral found coral → IntakeClosestObject, then go to step 1
5. IF ScanForCoral found nothing → DriveToPoint to a NEW location far away (at least 5+ meters):
   - Good exploration spots: (2,1,0), (14,1,0), (2,7,180), (14,7,180), (8,1,90), (8,7,90)
   - Pick a spot you haven't visited recently
   - After driving, go back to step 1
6. IF any reef < 2.5m → DriveToPoint away to safe area (y=1 or y=7) first

CRITICAL RULES:
- NEVER call StopRobot - always keep exploring and collecting coral
- Use ScanForCoral instead of manually rotating - it's one call and checks the whole circle
- If ScanForCoral returns coral_detected=false, you MUST move to a new location
- Move far distances (5+ meters) between scans to explore new areas
- Pick up coral immediately when detected or found by scan
- After picking up coral, check status again (there might be more nearby!)
- Keep track of where you've been and explore different areas

EXAMPLE WORKFLOW:
GetRobotStatus(no coral) → ScanForCoral(coral_detected=false) → DriveToPoint(14,1,0) → GetRobotStatus(no coral) → ScanForCoral(coral_detected=true) → IntakeClosestObject → GetRobotStatus → repeat

Always use ScanForCoral when you don't see coral! NEVER STOP - keep collecting coral!"""
    }
    
    # Agentic loop - keep calling tools until the AI is done
    max_iterations = 100
    iteration = 0
    
    try:
        while iteration < max_iterations:
            iteration += 1
            
            # Call OpenAI API
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[system_message] + conversation_history,
                tools=tools if tools else None,
                tool_choice="auto" if tools else None
            )
            
            message = response.choices[0].message
            
            # If AI wants to use tools
            if message.tool_calls:
                # Add assistant message with tool calls to history
                conversation_history.append({
                    "role": "assistant",
                    "content": message.content,
                    "tool_calls": [
                        {
                            "id": tc.id,
                            "type": "function",
                            "function": {
                                "name": tc.function.name,
                                "arguments": tc.function.arguments
                            }
                        } for tc in message.tool_calls
                    ]
                })
                
                # Execute each tool call
                for tool_call in message.tool_calls:
                    tool_name = tool_call.function.name
                    arguments = json.loads(tool_call.function.arguments)
                    
                    print(f"\nExecuting: {tool_name}({json.dumps(arguments)})")
                    
                    # Call the robot tool
                    result = call_robot_tool(tool_name, arguments)
                    print(f"Result: {json.dumps(result, indent=2)}")
                    
                    # Add tool result to conversation
                    conversation_history.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": json.dumps(result)
                    })
                
                # Continue the loop to let AI decide if it needs more tools
                print(f"\nAI is thinking... (iteration {iteration}/{max_iterations})")
                
            else:
                # No more tool calls, AI is done
                assistant_message = message.content
                conversation_history.append({"role": "assistant", "content": assistant_message})
                return assistant_message, conversation_history
        
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
        print("Warning: Cannot connect to robot MCP server at http://localhost:3000")
        print("   Make sure your robot simulation is running!")
        print()
    
    main()
