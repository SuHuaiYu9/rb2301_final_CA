import os, sys
os.chdir(sys.argv[1])
parts = open("reactive_parts.py").read()
helpers = parts.split("###HELPERS###\n")[1].split("###INIT###\n")[0]
init = parts.split("###INIT###\n")[1].split("###TIMER###\n")[0]
timer = parts.split("###TIMER###\n")[1]
t = open("template.py").read()
t = t.replace("class LidarNode(Node):", helpers.rstrip("\n") + "\n\nclass LidarNode(Node):")
t = t.replace("        self.pose = None\n", "        self.pose = None\n\n" + init.rstrip("\n") + "\n")
a = "        ###### INSERT CODE HERE ######\n        self.get_logger().info(f\"Pose: {self.pose}\")\n        self.move_2D(0.1)\n        ###### INSERT CODE HERE ######\n"
assert t.count(a) == 1
t = t.replace(a, "        ###### INSERT CODE HERE ######\n" + timer.rstrip("\n") + "\n        ###### INSERT CODE HERE ######\n")
open("claude_branch/claude_tools/reference/obstacle_course_reactive.py", "w").write(t)
open("rx/obstacle_course.py", "w").write(t)
