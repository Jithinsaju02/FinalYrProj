import pyttsx3
import time

# Initialize the TTS engine
engine = pyttsx3.init()

# Get all available voices
voices = engine.getProperty('voices')

# Set the desired voice (Tessa's voice ID)
for voice in voices:
    if voice.id == "com.apple.voice.compact.en-ZA.Tessa":
        engine.setProperty('voice', voice.id)
        break
else:
    print("Tessa's voice not found! Using default voice.")

# Set other properties
engine.setProperty('rate', 150)  # Set the speaking rate (default: 200 words/min)
engine.setProperty('volume', 0.9)  # Set the volume level (0.0 to 1.0)

# Main menu for text-to-speech options
def main_menu():
    print("\nText-to-Speech Program")
    print("1. Enter text manually")
    print("2. Read text from a file")
    print("3. Continuous mode (enter text repeatedly)")
    print("4. Exit")

while True:
    main_menu()
    choice = input("\nChoose an option (1, 2, 3, or 4): ")

    if choice == "1":
        # Option to enter text manually
        text = input("Enter the text to speak: ")
        if text:
            print("Speaking the text...")
            engine.say(text)
            engine.runAndWait()
            print("Done speaking.")

    elif choice == "2":
        # Option to read text from a file
        file_path = input("Enter the path to the text file: ")
        try:
            with open(file_path, "r") as file:
                text = file.read()
                print("Speaking the text from file...")
                engine.say(text)
                engine.runAndWait()
                print("Done speaking.")
        except FileNotFoundError:
            print("File not found. Please check the path and try again.")

    elif choice == "3":
        # Continuous mode for entering text repeatedly
        print("Enter text to convert to speech. Type 'exit' to stop continuous mode.")
        while True:
            user_input = input("Your text: ")
            if user_input.lower() == "exit":
                print("Exiting continuous mode.")
                break
            engine.say(user_input)
            engine.runAndWait()

    elif choice == "4":
        # Exit the program
        print("Exiting the program. Goodbye!")
        break

    else:
        print("Invalid choice. Please select a valid option.")
