import json

def process_command(input_json):
    # Parse the input JSON
    origin = input_json.get("origin")
    command = input_json.get("command")
    
    # Extract destination from the command intelligently
    # Assumes the destination is the last phrase after "to" or "is"
    keywords = ["to", "is"]
    destination = None
    for keyword in keywords:
        if keyword in command.lower():
            destination = command.lower().split(keyword)[-1].strip("?").strip()
            break

    # If no keywords match, set destination as None
    if not destination:
        destination = "Unknown"
    
    # Create the output JSON
    output_json = {
        "currentLocation": origin,
        "destination": destination
    }
    return output_json

# Example usage
input_json = {
    "origin": "LIBRARY",
    "command": "Where is foss lab?"
}

output_json = process_command(input_json)
print(json.dumps(output_json, indent=4))
