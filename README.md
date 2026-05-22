Upload Command:
sudo chmod a+rw /dev/ttyACM0                          main       
arduino-cli compile --fqbn esp32:esp32:esp32 /home/obsidian/Documents/stm/esp32_servo/esp32_servo.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32 /home/obsidian/Documents/stm/esp32_servo/esp32_servo.ino


