echo "Build Process"
rm -rf main
echo "Old Exe file deleted"
gcc -Wall -o main main.c  -lwiringPi -lpthread -lm
echo "Build Finished!"
sudo ./main 