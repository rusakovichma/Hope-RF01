@ECHO OFF
"D:\AVRS4\AvrAssembler2\avrasm2.exe" -S "D:\myAVR\RF01\labels.tmp" -fI -W+ie -C V2E -o "D:\myAVR\RF01\RF01.hex" -d "D:\myAVR\RF01\RF01.obj" -e "D:\myAVR\RF01\RF01.eep" -m "D:\myAVR\RF01\RF01.map" "D:\myAVR\RF01\RF01.asm"
