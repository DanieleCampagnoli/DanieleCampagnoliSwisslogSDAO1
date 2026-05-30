import tkinter as tk
from swarmmanager.bo import SwarmManagerObstacle, SwarmManagerRobotMovementLog,SwarmManagerRobot,SwarmManagerWarehouse
"""wrapper class for the tkinter library (visualization)"""
class SwarmManagerVisualization:
    def __init__(self, rectangle_size:int,max_x:int,max_y:int,get_movement_logs_callback):
        self.rectangle_size = rectangle_size
        self.max_x = max_x
        self.max_y=max_y
        self.root= tk.Tk()
        self.canvas = tk.Canvas(self.root, width=max_x*self.rectangle_size, height=max_y*self.rectangle_size)
        self.canvas.pack()
        self.iteration_number=1
        self.get_movement_logs_callback=get_movement_logs_callback

    """ generic function to draw a rectangle with a color and a tag """ 
    def draw_rectangle(self,x:int,y:int,color:str,tag:str):
        px = x * self.rectangle_size
        py = y * self.rectangle_size
        rect= self.canvas.create_rectangle(px, py, px+self.rectangle_size, py+self.rectangle_size, fill=color,tags=tag)
        return rect
    """ generic function to draw a label in the middle of the rectangle and a tag """
    def draw_rectangle_label(self,x:int,y:int,lbl:str,tag:str):
        px = x * self.rectangle_size
        py = y * self.rectangle_size
        rect_text=self.canvas.create_text(px+self.rectangle_size/2,py+self.rectangle_size/2,text=lbl,font=("Arial",14),tags=tag)
        return rect_text
    """ obstacles never change their positions """
    def draw_obstacles(self,obstacle: SwarmManagerObstacle):
        self.draw_rectangle(obstacle.coordinate.x, obstacle.coordinate.y,"gray","obstacle")
        self.draw_rectangle_label(obstacle.coordinate.x, obstacle.coordinate.y, f"{obstacle.coordinate.x}-{obstacle.coordinate.y} O{obstacle.obsId}","obstacle")
    """ robots  change their positions, this is the initial one """
    def draw_robot(self,log: SwarmManagerRobotMovementLog):
        self.draw_rectangle(log.next_x,log.next_y,"red","robot")        
        self.draw_rectangle_label(log.next_x,log.next_y, f"{log.next_x}-{log.next_y} R{log.robotId}","robot") 

    """ robot destination """
    def draw_robot_destination(self,robot: SwarmManagerRobot):
        self.draw_rectangle(robot.destinationPoint.x,robot.destinationPoint.y,"green","robotDest")        
        self.draw_rectangle_label(robot.destinationPoint.x,robot.destinationPoint.y, f"{robot.destinationPoint.x}-{robot.destinationPoint.y} R{robot.robotId} D","robotDest") 
    """draw a grid """
    def draw_grid(self,canvas, rows, cols, size):
        for r in range(rows):
            for c in range(cols):
                px = c * size
                py = r * size

                canvas.create_rectangle(
                    px, py,
                    px + size, py + size,
                    outline="gray"
                )

    """ start the visualization and call periodically the mode method """ 
    def start_visu(self,warehouse:SwarmManagerWarehouse):
        # draw static objects
        self.draw_grid(self.canvas, self.max_y, self.max_x, self.rectangle_size)
        for obs in warehouse.obstacles:
            self.draw_obstacles(obs)
        for robot in warehouse.robots:
            self.draw_robot_destination(robot)
        #draw moving objects (robots)
        self.move() 
        #start visu infinite loop
        self.root.mainloop()

    """move the robots"""        
    def move(self):
       # query the db and retrieve the all the logs (hopefully this does not kill the application)
       logs= self.get_movement_logs_callback(self.iteration_number)
       # if db is empty wait
       if len(logs)==0:
            return
       # Delete all rectangles and labels with tag "robot"
       self.canvas.delete("robot")
       # move the robots and draw the destinations
       for log in logs:
           self.draw_robot(log)
       self.iteration_number=self.iteration_number+1  
       #continue the movement after 1 sec
       self.root.after(1000, self.move)  

