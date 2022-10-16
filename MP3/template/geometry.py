# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by James Gao (jamesjg2@illinois.edu) on 9/03/2021
# Inspired by work done by Jongdeog Lee (jlee700@illinois.edu)

"""
This file contains geometry functions necessary for solving problems in MP3
"""

import math
import numpy as np
from alien import Alien
from typing import List, Tuple
from math import sqrt

# Borrwed from https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
# Function to return the minimum distance
# between a line segment AB and a point E
def minimalDistance(A, B, E) :
    # A, B, E are 2D points
    # AB is a line segment
    # E is a point
    # vector AB
    AB = [None, None]
    AB[0] = B[0] - A[0]
    AB[1] = B[1] - A[1]
 
    # vector BP
    BE = [None, None]
    BE[0] = E[0] - B[0]
    BE[1] = E[1] - B[1]
 
    # vector AP
    AE = [None, None]
    AE[0] = E[0] - A[0]
    AE[1] = E[1] - A[1]
 
    # Variables to store dot product
 
    # Calculating the dot product
    AB_BE = AB[0] * BE[0] + AB[1] * BE[1]
    AB_AE = AB[0] * AE[0] + AB[1] * AE[1]
 
    # Minimum distance from
    # point E to the line segment
    reqAns = 0
 
    # Case 1
    if (AB_BE > 0) :
 
        # Finding the magnitude
        # use np.linalg.norm()
        reqAns = np.linalg.norm(BE)
 
    # Case 2
    elif (AB_AE < 0) :
                                                         
            # Finding the magnitude
            # use np.linalg.norm()
            reqAns = np.linalg.norm(AE)

    # Case 3
    else:
 
        # Finding the perpendicular distance
        reqAns = abs(AB[0] * AE[1] - AB[1] * AE[0])
        # use np.linalg.norm() to find the magnitude of AB
        # reqAns = reqAns / np.linalg.norm(AB)


    return reqAns

# See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
# Given three collinear points p, q, r, the function checks if 
# point q lies on line segment 'pr' 
def onSegment(p, q, r):
    # print(p, q, r)
    if (
        (q[0] <= max(p[0], r[0]))
         and 
        (q[0] >= min(p[0], r[0])) 
         and
        (q[1] <= max(p[1], r[1]))
         and 
        (q[1] >= min(p[1], r[1]))
        ):
        return True
    return False


# See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
def orientation(p, q, r):
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
      
    # for details of below formula. 
    
    val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
    if (val > 0):
          
        # Clockwise orientation
        return 1
    elif (val < 0):
          
        # Counterclockwise orientation
        return 2
    else:
          
        # Collinear orientation
        return 0

# See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
# The main function that returns true if 
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):
    # Find the 4 orientations required for 
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    
    # print(o1, o2, o3, o4)
    # General case
    if ((o1 != o2) and (o3 != o4)):
        return True
  
    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if ((o1 == 0) and onSegment(p1, p2, q1)):
        print(p1, p2, q1)
        return True

    # p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if ((o2 == 0) and onSegment(p1, q2, q1)):
        return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if ((o3 == 0) and onSegment(p2, p1, q2)):
        return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if ((o4 == 0) and onSegment(p2, q1, q2)):
        return True

    # If none of the cases
    return False




def does_alien_touch_wall(alien, walls, granularity):
    """Determine whether the alien touches a wall

        Args:
            alien (Alien): Instance of Alien class that will be navigating our map
            walls (list): List of endpoints of line segments that comprise the walls in the maze in the format [(startx, starty, endx, endx), ...]
            granularity (int): The granularity of the map

        Return:
            True if touched, False if not
    """
    print(alien)
    alien_radius = alien.get_width()
    head, tail = alien.get_head_and_tail()
    # does_alien_touch_wall(alien, walls) with alien config [0, 100, 'Vertical'] returns False, expected: True
    for wall in walls:
        start = (wall[0], wall[1])
        end = (wall[2], wall[3])
        if doIntersect(start, end, head, tail):
            return True
        if minimalDistance(start, end, head) < alien_radius + granularity:
            return True
        if minimalDistance(start, end, tail) < alien_radius + granularity:
            return True
    return False
    


def does_alien_touch_goal(alien, goals):
    """Determine whether the alien touches a goal
        
        Args:
            alien (Alien): Instance of Alien class that will be navigating our map
            goals (list): x, y coordinate and radius of goals in the format [(x, y, r), ...]. There can be multiple goals
        
        Return:
            True if a goal is touched, False if not.
    """
    head, tail = alien.get_head_and_tail()
    for goal in goals:
        if minimalDistance(goal, head, tail) < goal[2]:
            return True
    return False



def is_alien_within_window(alien, window, granularity):
    """Determine whether the alien stays within the window
        
        Args:
            alien (Alien): Alien instance
            window (tuple): (width, height) of the window
            granularity (int): The granularity of the map
    """
    # Determine whether the alien stays within the window given a certain space granularity, returns True if it does and False otherwise.
    # within the window means that the alien's head and tail are both within the window
    # window is the boundary of the map
    # granularity is the size of the grid

    # is_alien_within_window(alien, window) with alien config [25.5, 25.5, 'Horizontal'] returns True, expected: False
    # is_alien_within_window(alien, window) with alien config [25.5, 25.5, 'Vertical'] returns True, expected: False
    # is_alien_within_window(alien, window) with alien config [194.5, 174.5, 'Horizontal'] returns True, expected: False
    # is_alien_within_window(alien, window) with alien config [194.5, 174.5, 'Vertical'] returns True, expected: False
    # is_alien_within_window(alien, window) with alien config [200, 100, 'Horizontal'] returns True, expected: False

    alien_radius = alien.get_width()
    head, tail = alien.get_head_and_tail()
    head_x, head_y = head
    tail_x, tail_y = tail
    window_width, window_height = window
    if head_x < granularity + alien_radius or head_x > window_width - granularity - alien_radius:
        return False
    if head_y < granularity + alien_radius or head_y > window_height - granularity - alien_radius:
        return False
    if tail_x < granularity + alien_radius or tail_x > window_width - granularity - alien_radius:
        return False
    if tail_y < granularity + alien_radius or tail_y > window_height - granularity - alien_radius:
        return False
    return True




def point_segment_distance(point, segment):
    """Compute the distance from the point to the line segment.
    Hint: Lecture note "geometry cheat sheet"

        Args:
            point: A tuple (x, y) of the coordinates of the point.
            segment: A tuple ((x1, y1), (x2, y2)) of coordinates indicating the endpoints of the segment.

        Return:
            Euclidean distance from the point to the line segment.
    """
    # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    x, y = point
    x1, y1 = segment[0]
    x2, y2 = segment[1]
    if x1 == x2 and y1 == y2:
        return np.linalg.norm(np.array([x,y]) - np.array([x1,y1]))
    u = ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
    if u > 1:
        u = 1
    elif u < 0:
        u = 0
    x0 = x1 + u * (x2 - x1)
    y0 = y1 + u * (y2 - y1)
    return np.linalg.norm(np.array([x,y]) - np.array([x0,y0]))


def do_segments_intersect(segment1, segment2):
    """Determine whether segment1 intersects segment2.  
    We recommend implementing the above first, and drawing down and considering some examples.
    Lecture note "geometry cheat sheet" may also be handy.

        Args:
            segment1: A tuple of coordinates indicating the endpoints of segment1.
            segment2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            True if line segments intersect, False if not.
    """
    # use doIntersect() to determine whether the segments intersect
    # return the result
    x1, y1 = segment1[0]
    x2, y2 = segment1[1]
    x3, y3 = segment2[0]
    x4, y4 = segment2[1]
    return doIntersect((x1, y1), (x2, y2), (x3, y3), (x4, y4))


def segment_distance(segment1, segment2):
    """Compute the distance from segment1 to segment2.  You will need `do_segments_intersect`.
    Hint: Distance of two line segments is the distance between the closest pair of points on both.

        Args:
            segment1: A tuple of coordinates indicating the endpoints of segment1.
            segment2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            Euclidean distance between the two line segments.
    """
    # if the segments intersect, return 0
    # otherwise, return the minimum distance between the two segments
    if do_segments_intersect(segment1, segment2):
        return 0
    return min(point_segment_distance(segment1[0], segment2), point_segment_distance(segment1[1], segment2), point_segment_distance(segment2[0], segment1), point_segment_distance(segment2[1], segment1))


if __name__ == '__main__':

    from geometry_test_data import walls, goals, window, alien_positions, alien_ball_truths, alien_horz_truths, \
        alien_vert_truths, point_segment_distance_result, segment_distance_result, is_intersect_result

    # Here we first test your basic geometry implementation
    def test_point_segment_distance(points, segments, results):
        num_points = len(points)
        num_segments = len(segments)
        for i in range(num_points):
            p = points[i]
            for j in range(num_segments):
                seg = ((segments[j][0], segments[j][1]), (segments[j][2], segments[j][3]))
                cur_dist = point_segment_distance(p, seg)
                assert abs(cur_dist - results[i][j]) <= 10 ** -3, \
                    f'Expected distance between {points[i]} and segment {segments[j]} is {results[i][j]}, ' \
                    f'but get {cur_dist}'


    def test_do_segments_intersect(center: List[Tuple[int]], segments: List[Tuple[int]],
                                   result: List[List[List[bool]]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    if do_segments_intersect(a, b) != result[i][j][k]:
                        if result[i][j][k]:
                            assert False, f'Intersection Expected between {a} and {b}.'
                        if not result[i][j][k]:
                            assert False, f'Intersection not expected between {a} and {b}.'


    def test_segment_distance(center: List[Tuple[int]], segments: List[Tuple[int]], result: List[List[float]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    distance = segment_distance(a, b)
                    assert abs(result[i][j][k] - distance) <= 10 ** -3, f'The distance between segment {a} and ' \
                                                                  f'{b} is expected to be {result[i]}, but your' \
                                                                  f'result is {distance}'

    def test_helper(alien: Alien, position, truths):
        alien.set_alien_pos(position)
        config = alien.get_config()

        touch_wall_result = does_alien_touch_wall(alien, walls, 0)
        # touch_goal_result = does_alien_touch_goal(alien, goals)
        # in_window_result = is_alien_within_window(alien, window, 0)

        assert touch_wall_result == truths[
            0], f'does_alien_touch_wall(alien, walls) with alien config {config} returns {touch_wall_result}, ' \
                f'expected: {truths[0]}'
        # assert touch_goal_result == truths[
        #     1], f'does_alien_touch_goal(alien, goals) with alien config {config} returns {touch_goal_result}, ' \
        #         f'expected: {truths[1]}'
        # assert in_window_result == truths[
        #     2], f'is_alien_within_window(alien, window) with alien config {config} returns {in_window_result}, ' \
        #         f'expected: {truths[2]}'


    # Initialize Aliens and perform simple sanity check.
    alien_ball = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Ball', window)
    test_helper(alien_ball, alien_ball.get_centroid(), (False, False, True))

    alien_horz = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal', window)
    test_helper(alien_horz, alien_horz.get_centroid(), (False, False, True))

    alien_vert = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical', window)
    test_helper(alien_vert, alien_vert.get_centroid(), (True, False, True))

    edge_horz_alien = Alien((50, 100), [100, 0, 100], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal',
                            window)
    edge_vert_alien = Alien((200, 70), [120, 0, 120], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical',
                            window)

    centers = alien_positions
    segments = walls
    test_point_segment_distance(centers, segments, point_segment_distance_result)
    test_do_segments_intersect(centers, segments, is_intersect_result)
    test_segment_distance(centers, segments, segment_distance_result)

    for i in range(len(alien_positions)):
        test_helper(alien_ball, alien_positions[i], alien_ball_truths[i])
        print(f'Alien Ball Test {i + 1} Passed')
        test_helper(alien_horz, alien_positions[i], alien_horz_truths[i])
        print(f'Alien Horizontal Test {i + 1} Passed')
        test_helper(alien_vert, alien_positions[i], alien_vert_truths[i])
        print(f'Alien Vertical Test {i + 1} Passed')

    # Edge case coincide line endpoints
    test_helper(edge_horz_alien, edge_horz_alien.get_centroid(), (True, False, False))
    print(f'Edge Case Alien Horizontal Test Passed')
    test_helper(edge_horz_alien, (110, 55), (True, True, True))
    print(f'Edge Case Alien Horizontal Test Passed')
    test_helper(edge_vert_alien, edge_vert_alien.get_centroid(), (True, False, True))
    print(f'Edge Case Alien Vertical Test Passed')

    print("Geometry tests passed\n")
