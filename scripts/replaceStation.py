#!/usr/bin/python3
import os
import re

# Get path
directory = os.path.join(os.getcwd(), "hiwonder-toolbox")
# Sentences for patterns
sentenceSSID = "HW_WIFI_STA_SSID = "
sentencePASSWORD = "HW_WIFI_STA_PASSWORD = "
sentenceMode = "HW_WIFI_MODE = "

def main():
    # User input to desired mode
    os.system('clear')
    print("Please input the number for the desired mode: ")
    print("1. AP mode \n2. STA Client mode \n3. STA Client w/eht0 internet share")
    mode = int(input("Mode: "))
    mode if 0 < mode < 4 else 1

    os.system('clear')
    # User input SSID and Password
    print("Please input the new SSID and password: ")
    ssid = str(input("SSID: "))
    password = str(input("Password: "))

    os.system('clear')
    # File to modificate
    f = os.path.join(directory, "hw_wifi.py")
    if os.path.isfile(f):
        with open(f, 'r') as file:
            filedata = file.read()
            # Replace network configuration
            pattern = re.compile(rf'({re.escape(sentenceMode)})(\d+)') # Regular expression for mode
            filedata = pattern.sub(sentenceMode + f'{mode}', filedata) # Replace mode

            pattern = re.compile(rf'({re.escape(sentenceSSID)}")([^"]*)(")') # Regular expression for ssid
            currSSID = pattern.search(filedata).group(2) # Current ssid
            filedata = pattern.sub(sentenceSSID + f'"{ssid}"', filedata) # Replace ssid

            pattern = re.compile(rf'({re.escape(sentencePASSWORD)}")([^"]*)(")') # Regular expression for password
            currPASS = pattern.search(filedata).group(2) # Current password
            filedata = pattern.sub(sentencePASSWORD + f'"{password}"', filedata) # Replace password
            file.close()

        # Write modifications in the original file
        with open(f, 'w') as file:
            file.write(filedata)
            file.close()

        # Create file with past configurations
        configFile = os.path.join(os.getcwd(), "PastNetworkConfig")
        with open(configFile, 'w') as file:
            file.write("SSID: " + currSSID + "\n")
            file.write("PASSWORD: " + currPASS)
            file.close()
        
        # Display information about the modifications
        if (mode == 1):
            print("Changed to AP Mode")
        elif (mode == 2):
            print("Changed to STA Client Mode")
        else:
            print("Changed to STA Client w/eht0 internet share")
        print(f"\nDiscarded SSID and Password saved in {configFile}")
        print("----------Please remember to reboot in order to apply changes----------")

if __name__ == "__main__":
    main() # Get data from the command line
