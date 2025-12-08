# Intent.msg - Custom message for parsed voice command intents
# Part of vla_control package for Module 4: Vision-Language-Action

# Header with timestamp
std_msgs/Header header

# Action type (e.g., "navigate", "pick", "place", "search", "deliver")
string action_type

# Target object (e.g., "red cup", "book", "bottle")
string target_object

# Target location (e.g., "kitchen", "living room", "table")
string target_location

# Source location for fetch tasks (e.g., "kitchen counter")
string source_location

# Constraints (JSON string for flexibility)
# Example: {"color": "red", "size": "large", "urgent": true}
string constraints

# Confidence score from speech recognition (0.0 to 1.0)
float32 confidence

# Original transcribed text from Whisper
string raw_text
