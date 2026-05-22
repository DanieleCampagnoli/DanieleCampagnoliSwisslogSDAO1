import tkinter as tk
import random

SIZE = 50
ROWS, COLS = 10, 10

root = tk.Tk()
canvas = tk.Canvas(root, width=COLS*SIZE, height=ROWS*SIZE)
canvas.pack()

x, y = 0, 0
rect = canvas.create_rectangle(0, 0, SIZE, SIZE, fill="red")

def move():
    global x, y
    dx = random.choice([-1, 0, 1])
    dy = random.choice([-1, 0, 1])

    x = (x + dx) % COLS
    y = (y + dy) % ROWS

    canvas.coords(rect, x*SIZE, y*SIZE, (x+1)*SIZE, (y+1)*SIZE)
    root.after(1000, move)

move()
root.mainloop()