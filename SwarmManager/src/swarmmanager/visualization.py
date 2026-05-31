import tkinter as tk
from swarmmanager.bo import (
    SwarmManagerObstacle,
    SwarmManagerRobotMovementLog,
    SwarmManagerRobot,
    SwarmManagerWarehouse
)

class SwarmManagerVisualization:
    def __init__(self, rectangle_size: int, max_x: int, max_y: int, get_movement_logs_callback):
        self.rectangle_size = rectangle_size
        self.max_x = max_x
        self.max_y = max_y
        self.get_movement_logs_callback = get_movement_logs_callback
        self.iteration_number = 1

        # Root
        self.root = tk.Tk()
        self.root.title("Swarm Visualization")

        # MAIN container
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Canvas inside main frame
        self.canvas = tk.Canvas(main_frame, bg="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Vertical scrollbar (right of canvas)
        vbar = tk.Scrollbar(main_frame, orient=tk.VERTICAL)
        vbar.pack(side=tk.RIGHT, fill=tk.Y)
        vbar.config(command=self.canvas.yview)

        # Horizontal scrollbar (bottom of canvas)
        hbar = tk.Scrollbar(self.root, orient=tk.HORIZONTAL)
        hbar.pack(side=tk.BOTTOM, fill=tk.X)
        hbar.config(command=self.canvas.xview)

        self.canvas.config(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

        # ✅ Status bar (true bottom, full width)
        self.status_label = tk.Label(
            self.root,
            text="x=0, y=0",
            anchor="w",
            bg="#222",
            fg="white",
            padx=8
        )
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)

        # zoom level value
        self.zoom_level = 1.0

        # ----------- Interaction (PRO UX) -----------

        # Pan (click + drag)
        self.canvas.bind("<ButtonPress-1>", self.start_pan)
        self.canvas.bind("<B1-Motion>", self.do_pan)

        # Zoom
        self.canvas.bind("<MouseWheel>", self.zoom)   # Windows
        self.canvas.bind("<Button-4>", self.zoom)     # Linux up
        self.canvas.bind("<Button-5>", self.zoom)     # Linux down

        # Optional keyboard navigation
        self.root.bind("<Up>", lambda e: self.canvas.yview_scroll(-1, "units"))
        self.root.bind("<Down>", lambda e: self.canvas.yview_scroll(1, "units"))
        self.root.bind("<Left>", lambda e: self.canvas.xview_scroll(-1, "units"))
        self.root.bind("<Right>", lambda e: self.canvas.xview_scroll(1, "units"))
        # mouse movement
        self.canvas.bind("<Motion>", self.on_mouse_move)


    # ----------- Interaction Methods -----------
    def on_mouse_move(self, event):
        # Convert screen coords → canvas coords (important for zoom + pan)
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)

        # Convert to grid cell
        grid_x = int(canvas_x // self.rectangle_size)
        grid_y = int(canvas_y // self.rectangle_size)

        # Update label
        self.status_label.config(text=f"x={grid_x}, y={grid_y}| zoom={self.zoom_level:.2f}")

    def start_pan(self, event):
        self.canvas.scan_mark(event.x, event.y)

    def do_pan(self, event):
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    def zoom(self, event):
        scale = 1.1

        if hasattr(event, "delta"):
            if event.delta < 0:
                scale = 0.9
        elif event.num == 5:
            scale = 0.9

        # mouse position in canvas coords
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)

        self.canvas.scale("all", x, y, scale, scale)
        
        # track zoom level   
        self.zoom_level *= scale

        # ✅ Show / Hide labels
        if self.zoom_level < 0.5:
            self.canvas.itemconfigure("label", state="hidden")
        else:
            self.canvas.itemconfigure("label", state="normal")
        
        # update scroll region
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))


    # ----------- Drawing -----------

    def draw_rectangle(self, x: int, y: int, color: str, tag: str):
        px = x * self.rectangle_size
        py = y * self.rectangle_size
        return self.canvas.create_rectangle(
            px, py,
            px + self.rectangle_size,
            py + self.rectangle_size,
            fill=color,
            outline="",
            tags=tag
        )

    def draw_rectangle_label(self, x: int, y: int, lbl: str, tag: str):
        # Only draw labels if zoomed enough (avoid clutter)
        if self.rectangle_size < 8:
            return

        px = x * self.rectangle_size
        py = y * self.rectangle_size

        return self.canvas.create_text(
            px + self.rectangle_size / 2,
            py + self.rectangle_size / 2,
            text=lbl,
            font=("Arial", 10),
            tags=(tag, "label") 
        )

    def draw_obstacles(self, obstacle: SwarmManagerObstacle):
        self.draw_rectangle(obstacle.coordinate.x, obstacle.coordinate.y, "gray", "obstacle")
        self.draw_rectangle_label(
            obstacle.coordinate.x,
            obstacle.coordinate.y,
            f"O{obstacle.obsId}",
            "obstacle"
        )

    def draw_robot(self, log: SwarmManagerRobotMovementLog):
        self.draw_rectangle(log.next_x, log.next_y, "red", "robot")
        self.draw_rectangle_label(
            log.next_x, log.next_y,
            f"R{log.robotId}",
            "robot"
        )

    def draw_robot_destination(self, robot: SwarmManagerRobot):
        self.draw_rectangle(
            robot.destinationPoint.x,
            robot.destinationPoint.y,
            "green",
            "robotDest"
        )
        self.draw_rectangle_label(
            robot.destinationPoint.x,
            robot.destinationPoint.y,
            f"D{robot.robotId}",
            "robotDest"
        )

    def draw_grid(self, canvas, rows, cols, size):
        # Lightweight grid (lines instead of 40k rectangles)
        for c in range(cols + 1):
            x = c * size
            canvas.create_line(x, 0, x, rows * size, fill="#eee")

        for r in range(rows + 1):
            y = r * size
            canvas.create_line(0, y, cols * size, y, fill="#eee")

    # ----------- Main Flow -----------

    def start_visu(self, warehouse: SwarmManagerWarehouse):
        # Draw static objects
        self.draw_grid(self.canvas, self.max_y, self.max_x, self.rectangle_size)

        for obs in warehouse.obstacles:
            self.draw_obstacles(obs)

        for robot in warehouse.robots:
            self.draw_robot_destination(robot)

        # Set scrollable area
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        # Start movements
        self.move()

        self.root.mainloop()

    def move(self):
        logs = self.get_movement_logs_callback(self.iteration_number)

        if len(logs) == 0:
            self.root.after(1000, self.move)
            return

        # Remove old robots
        self.canvas.delete("robot")

        # Draw new positions
        for log in logs:
            self.draw_robot(log)

        self.iteration_number += 1

        # Keep scroll region updated
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

        self.root.after(1000, self.move)