import shutil
import os

profilePath = os.environ.get('USERPROFILE')
if profilePath != None:
    libPath = profilePath + '/Documents/Arduino/libraries/Dynamino'
    shutil.copy2("Dynamino.h", libPath)
    shutil.copy2("Dynamino.cpp", libPath)
else:
    print("Couldn't find user profile")
