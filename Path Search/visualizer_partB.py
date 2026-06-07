######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

import math
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

class GUI_partB:

    def __init__(self, test_case, path, smooth_path, obs, benchmark_path, converted_path):
        # Warehouse and path information
        self.test_case = test_case
        self.path = path
        self.smooth_path = smooth_path
        self.obs = obs
        self.benchmark_path = benchmark_path
        self.converted_path = converted_path

        # Calculated information
        self.dist, self.idx, (self.p1, self.p2), self.path_resampled, self.converted_resampled = self.max_divergence(
                                                                                         self.path,
                                                                                         self.converted_path,
                                                                                         num_samples=200) #200
        self.turn_angle, self.turn_idx, self.turn_point = self.max_turn_angle(self.converted_path)
        self.matches = [pt for i, pt in enumerate(self.converted_path) if pt in self.obs]
        self.path_len = len(self.converted_path)

    def cumulative_lengths(self, points):
        lengths = [0.0]
        for i in range(1, len(points)):
            prev, curr = points[i - 1], points[i]
            seg_len = math.dist(prev, curr)
            lengths.append(lengths[-1] + seg_len)
        return lengths

    def interpolate_point(self, points, cum_lengths, target_len):
        for i in range(1, len(points)):
            if cum_lengths[i] >= target_len:
                t = (target_len - cum_lengths[i - 1]) / (cum_lengths[i] - cum_lengths[i - 1])
                y = points[i - 1][0] + t * (points[i][0] - points[i - 1][0])
                x = points[i - 1][1] + t * (points[i][1] - points[i - 1][1])
                return [y, x]
        return points[-1]

    def resample_curve(self, points, num_samples=200):
        cum_lengths = self.cumulative_lengths(points)
        total_length = cum_lengths[-1]
        step = total_length / (num_samples - 1)
        return [self.interpolate_point(points, cum_lengths, i * step) for i in range(num_samples)]

    def max_divergence(self, path, converted_path, num_samples=200): #100
        path_resampled = self.resample_curve(path, num_samples)
        converted_resampled = self.resample_curve(converted_path, num_samples)

        max_dist = 0
        index = -1
        for i, (p1, p2) in enumerate(zip(path_resampled, converted_resampled)):
            dist = math.dist(p1, p2)
            if dist > max_dist:
                max_dist = dist
                index = i
        return max_dist, index, (path_resampled[index], converted_resampled[index]), path_resampled, converted_resampled

    def max_turn_angle(self, points):
        """Compute maximum turn angle (in degrees) for a polyline."""
        max_angle = 0
        index = -1
        for i in range(1, len(points) - 1):
            p0, p1, p2 = points[i - 1], points[i], points[i + 1]
            v1 = [p1[0] - p0[0], p1[1] - p0[1]]
            v2 = [p2[0] - p1[0], p2[1] - p1[1]]

            len1, len2 = math.dist(p0, p1), math.dist(p1, p2)
            if len1 == 0 or len2 == 0:
                continue

            dot = v1[0] * v2[0] + v1[1] * v2[1]
            cos_theta = max(-1, min(1, dot / (len1 * len2)))  # clamp for safety
            angle = math.degrees(math.acos(cos_theta))

            if angle > max_angle:
                max_angle = angle
                index = i
        return max_angle, index, points[index]

    #VERBOSE = TRUE
    def print_debug(self):
        print("TEST CASE " + str(self.test_case))
        print("Original Path:")
        print(self.path)
        if self.smooth_path != self.converted_path:
            print("Path submitted does NOT match moves submitted.")
            print("Smooth Path (submitted by student):")
            print(self.smooth_path)
            print("Converted Path (created from student submitted moves):")
            print(self.converted_path)
        else:
            print("Smooth Path matches your submitted moves.")
            print("Smooth Path:")
            print(self.converted_path)
        print("")
        if self.dist > 3:
            print("Max Divergence from Original Path is too high.  Divergence needs to be 3 or less.  "
                  "Turn on Visualizer to see where max divergence occurred.")
        else:
            print("Max Divergence from Original Path is acceptable.")
        print(f"Max divergence = {self.dist:.2f} at line sample {self.idx} between points [{self.p1[0]:.2f}, {self.p1[1]:.2f}] and [{self.p2[0]:.2f}, {self.p2[1]:.2f}]")
        print("")
        if round(self.turn_angle,2) > 45:
            print("Max Turn Radius too high.  Max angle needs to be 45 deg or less.  "
                  "Turn on Visualizer to see where max angle occurred.")
        else:
            print("Max Turn Radius is acceptable.")
        print(f"Max turn angle = {self.turn_angle:.1f}° at point {self.turn_point} (index {self.turn_idx})")
        print("")
        if len(self.smooth_path) > self.benchmark_path or len(self.converted_path) > self.benchmark_path:
            print("Path submitted is longer than the benchmark path. Make sure you are not including duplicate points in your path, "
                  "and/or you may want to increase smoothing.")
        else:
            print("Path submitted meets or beats the benchmark.")
        print("Your path: " + str(max(len(self.smooth_path), len(self.converted_path))) + " vs. Benchmark: " + str(self.benchmark_path))
        print("")
        if self.matches:
            print("You have " + str(len(self.matches)) + " collisions along your path.")
        else:
            print("You have no collisions along your path.")
        print("")

    #calc grade based on debug parameters
    def grading(self):
        self.grade = 0
        temp_grade = 0
        if (self.path[-1] == self.converted_path[-1] and self.path[0] == self.converted_path[0] and self.path != self.smooth_path):
            if not self.matches:
                if round(self.dist, 2) <= 3:
                    temp_grade += 1
                if round(self.turn_angle, 2) <= 45.00:
                    temp_grade += 1
                if self.path_len <= self.benchmark_path and self.smooth_path == self.converted_path:
                    temp_grade += 1
        if temp_grade == 3:
            self.grade = 1.0
        if temp_grade == 2:
            self.grade = 0.5

        return self.grade

    #VISULIZER = TRUE   fig, ax = plt.subplots()
    def draw_graph(self):
        #y,x to x,y coords (matplotlib only works with x,y)
        path_y, path_x = zip(*self.path_resampled)
        converted_y, converted_x = zip(*self.converted_resampled)

        plt.figure(figsize=(5, 5.7))#5.7))
        plt.plot(path_x, path_y, color='cyan', linestyle='-', label="Path")
        plt.plot(converted_x, converted_y, color='purple', linestyle='--', label="Path from Moves")

        if self.smooth_path != self.converted_path:
            plt.plot(converted_x, converted_y, color='red', linestyle='-', label="Student Path not match moves")

        # Add obstacles
        obs_y, obs_x = zip(*self.obs)
        plt.scatter(obs_x, obs_y, marker='s', color='black', s=100, zorder=6, label="Obstacles")
        if self.matches:
            match_y, match_x = zip(*self.matches)
        else:
            match_x = -5
            match_y = -5
        plt.scatter(match_x, match_y, marker='X', color='r', s=100, zorder=6, label="Crashes")
        #fig, ax = plt.subplots()  #new
        #ax.scatter(match_x, match_y, marker='X', color='r', s=100, zorder=6, label="Crashes") #new
        #ax.xaxis.tick_top()  # Moves the tick marks to the top #new
        #ax.xaxis.set_label_position('top')  # Moves the x-axis label to the top #new

        # Mark max divergence
        self.p1 = self.p1[::-1] #swap y,x to x,y
        self.p2 = self.p2[::-1] #swap y,x to x,y
        plt.scatter(*self.p1, color='orange', s=80, zorder=5)
        plt.scatter(*self.p2, color='orange', s=80, zorder=5)
        plt.plot([self.p1[0], self.p2[0]], [self.p1[1], self.p2[1]],
                 color='orange',
                 linestyle=':',
                 label=f"Max divergence = {self.dist:.2f}")

        # Mark max turn angle
        plt.scatter(*self.turn_point[::-1],
                    color='green',
                    s=100,
                    marker='<',
                    zorder=6,
                    label=f"Max turn = {self.turn_angle:.1f}°")

        # --- Force 1x1 grid ---
        ax = plt.gca()

        # Normal ticks at whole numbers
        ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(1))

        # Draw grid lines at half steps (0.5, 1.5, 2.5, ...)
        ax.set_xticks([x + 0.5 for x in ax.get_xticks()], minor=True)
        ax.set_yticks([y + 0.5 for y in ax.get_yticks()], minor=True)

        # Show only the major ticks (whole numbers) as labels
        ax.tick_params(which='major', labelbottom=True, labelleft=True)
        ax.tick_params(which='minor', labelbottom=False, labelleft=False)

        # Grid lines follow the minor ticks (offset by 0.5)
        ax.grid(which='major', linewidth=0.1, color='white', linestyle='-')
        ax.grid(which='minor', linewidth=0.8, color='gray', linestyle='-')

        plt.subplots_adjust(bottom=0.2)  # more space at bottom

        # Grab all handles and labels in the order they were created
        handles, labels = ax.get_legend_handles_labels()

        # Reorder by index
        order = [0, 4, 2, 1, 5, 3]

        # Apply order
        handles = [handles[i] for i in order]
        labels = [labels[i] for i in order]

        # Place legend outside the graph
        plt.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2)


        #plt.xlabel("X") #old
        plt.ylabel("Y")
        plt.title("Test Case " + str(self.test_case) + ": Path vs Smooth Path", pad=0)#20)
        plt.grid(True)
        plt.axis("equal")
        # ax.autoscale(enable=False)
        # ax.set_xmargin(0)
        # ax.set_ymargin(0.5)
        if self.test_case == 1 or self.test_case == 2 or self.test_case == 11 or self.test_case == 12:
            plt.xlim(left=-0.5, right=3.5)
            #plt.ylim(bottom=-0.5, top=3.5) #old
            plt.ylim(top=-0.5, bottom=3.5) # Revision to flip y-axis (0,0) at top left ***
        else:
            plt.xlim(left=-0.5, right=9.5)
            #plt.ylim(bottom=0, top=9) #old
            plt.ylim(top=0, bottom=9)# Revision to flip y-axis (0,0) at top left ***

        # Move the x-axis tick labels and ticks to the top
        ax.xaxis.set_ticks_position('top')
        # Set the position for the label "X" (which will follow the axis to the top)
        ax.xaxis.set_label_position('top')
        # Set the label text
        ax.set_xlabel("X")

        #uncomment below line to save plot
        #plt.savefig("Path Search (Test Case " + str(self.test_case) + ").png", dpi=300, bbox_inches="tight")
        #plt.show(dpi=300, bbox_inches="tight")
        plt.show()

if __name__ == '__main__':
    test_case = 1
    path = [[0, 0], [0, 1], [0, 2], [1, 2]]
    smooth_path = [[0, 0], [0, 1], [1, 2]]
    converted_path = smooth_path
    obs = [[3, 3]]
    benchmark_path = 3

    gui = GUI_partB(test_case, path, smooth_path, obs, benchmark_path,converted_path)
    gui.draw_graph()

    gui.print_debug()
