import os
import sys

# Today record the study of os/sys
current_path = os.getcwd()
print(current_path)
print(sys.path[0])
print(sys.version)

# os.chdir("/tmp")
# print(f"change: {os.getcwd()}")

path = os.path.join("data", "images")
print(path)

abs_path = os.path.abspath("train.py")
print(abs_path)
print(__file__)
print(__name__)
