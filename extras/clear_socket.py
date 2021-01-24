import os
print("Clearing TCP Socket 11411")
host = os.system("fuser -k 11411/tcp")
print("Socket Cleared. Good to go !")
