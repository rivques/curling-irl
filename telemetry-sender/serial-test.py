import sys

print("What is the Answer to the Ultimate Question of Life, the Universe, and Everything?")
while True:
    char = sys.stdin.read(1)
    if char:
        print(f"Received: {char.strip()}")