# 🚛 SDAO Dev Challenge 2026: The Swarm Gridlock

Welcome to the Swarm Gridlock challenge! Your mission is to route a fleet of automated warehouse robots from their starting positions to their destinations as efficiently as possible, without crashing them into walls or each other.

This README explains how to read the provided datasets, how the movement mechanics work, and how to format your solution.

---

## 📂 1. The Input Files (`.in`)

Each `.in` file represents a specific warehouse scenario. The file is plain text and follows a strict line-by-line format.

### The Coordinate System
*   The grid is represented by `(X, Y)` coordinates.
*   `(0, 0)` is the **top-left** corner of the warehouse.
*   **X** represents the column (increases as you move **Right**).
*   **Y** represents the row (increases as you move **Down**).

### File Structure
The file is structured as follows:

1.  **Line 1:** `[Grid_Width] [Grid_Height]`
2.  **Line 2:** `[Total_Number_of_Obstacles]`
3.  **Next N Lines:** `[Obstacle_X] [Obstacle_Y]` (One line per obstacle)
4.  **Next Line:** `[Total_Number_of_Robots]`
5.  **Next M Lines:** `[Start_X] [Start_Y] [End_X] [End_Y]` (One line per robot)

### Example `dataset_A.in` Explained
```text
10 10        <-- The warehouse is 10 spaces wide (X: 0-9) and 10 spaces tall (Y: 0-9)
2            <-- There are 2 obstacles (walls/shelves) in this grid
4 5          <-- Obstacle 1 is at X=4, Y=5
4 6          <-- Obstacle 2 is at X=4, Y=6
2            <-- There are 2 robots that need routing
0 0 9 9      <-- Robot 0 starts at (0,0) and needs to reach (9,9)
9 0 0 9      <-- Robot 1 starts at (9,0) and needs to reach (0,9)

(Note: Robot IDs are implicitly assigned starting from 0, based on the order they appear in the file).

🛠️ 2. The Output Files (.out)
Your program must generate a .out file containing the step-by-step movements for every robot.

Output Formatting Rules
The file must contain exactly one line per robot, in the same order they appeared in the input file.

Each line should be a string of characters representing that robot's movement per tick.

Spaces between characters are optional (the validator ignores them), but they help with readability.

Valid Movement Commands
U (Up): Y decreases by 1

D (Down): Y increases by 1

L (Left): X decreases by 1

R (Right): X increases by 1

W (Wait): Robot stays in its current position

Example dataset_A.out
Plaintext
R R R D D D W R R D D D D R R R W W W D R
L L L D D D D L L D D D D L L L L W W W W
⏱️ 3. The Rules of Physics (Very Important!)
The simulation operates in discrete time steps called Ticks. 1 Tick = 1 Move Command.

Simultaneous Movement
All robots execute their commands at the exact same time during a tick. This leads to two critical movement rules:

✅ VALID: "Follow the Leader"
Because movement is simultaneous, robots can safely tailgate each other.
If Robot A leaves a space in the same exact tick that Robot B enters it, no collision occurs!

Plaintext
Tick 0: [Robot A] [Robot B] [ Empty ]
        (Both move Right)
Tick 1: [ Empty ] [Robot A] [Robot B] --> PERFECTLY SAFE!
❌ INVALID: "The Swap" (Head-on Collision)
Robots cannot pass through each other. If Robot A tries to move into Robot B's old space, and Robot B tries to move into Robot A's old space at the same time, they crash on the edge between the two tiles.

Plaintext
Tick 0: [Robot A] [Robot B] 
        (A moves Right, B moves Left)
Tick 1:     💥 CRASH! 💥    --> FATAL ERROR!
Destination Arrival
The simulation does not end until EVERY robot has reached its final destination. If Robot 0 reaches its destination in 15 ticks, but Robot 1 takes 30 ticks, you must issue 15 W (Wait) commands for Robot 0 to keep it parked safely while Robot 1 finishes.

Your final score is the total number of ticks required to get the last robot home.

🚦 4. How to use the Validator
We have provided a local Java tool (Validator.jar) so you can test your solutions offline before submitting them to the SDAO leaderboard.

To test your output, open your terminal and run:

Bash
java -jar Validator.jar dataset_A.in dataset_A.out
What the Validator does:

Verifies no robots drove out of bounds.

Verifies no robots hit an obstacle.

Verifies no vertex collisions (two robots ending up on the same square).

Verifies no edge collisions (two robots swapping places).

Checks if every robot ended up exactly on its target destination.

Outputs your final Score (Total Ticks)!

Good luck, and Happy Hacking!
- The SDAO Team