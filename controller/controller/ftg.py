#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node


class FTG_Controller(Node):
    #Lidar processing params
    PREPROCESS_CONV_SIZE = 3
    
    #Steering params
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees
    MILD_CURVE_ANGLE = np.pi / 6  # 30 degrees
    ULTRASTRAIGHTS_ANGLE = np.pi / 60  # 3 deg

    def __init__(self,
                 mapping,
                 debug,
                 safety_radius,
                 max_lidar_dist,
                 max_speed,
                 range_offset,
                 track_width,
                 **adv) -> None:
        super().__init__('controller_manager')
        """
        Initialize the FTG controller.

        Parameters:
            mapping (bool): Flag indicating whether FTG is used for mapping or not.
        """
        self.mapping = mapping
        self.DEBUG = debug
        self.SAFETY_RADIUS = safety_radius
        self.MAX_LIDAR_DIST = max_lidar_dist
        self.MAX_SPEED = max_speed
        self.radians_per_elem = None # used when calculating the angles of the LiDAR data
        self.range_offset = range_offset
        self.track_width = track_width
        
        
        # Speed params
        self._apply_adv(adv)
        self._set_speed_tiers()
        
        
        self.velocity = 0
        self.scan = None
        self.last_target_idx = None
        self.last_d_free = None

        self.best_pnt = self.create_publisher(Marker, '/best_points/marker', 10)
        self.scan_pub = self.create_publisher(MarkerArray, '/scan_proc/markers', 10)
        self.best_gap = self.create_publisher(MarkerArray, '/best_gap/markers', 10)

    def update_params(self, debug, safety_radius, max_lidar_dist, max_speed, range_offset, track_width, **adv) -> None:
        """Live update from dynamic reconfigure (controller_manager.l1_param_cb)."""
        self.DEBUG = debug
        self.SAFETY_RADIUS = safety_radius
        self.MAX_LIDAR_DIST = max_lidar_dist
        self.MAX_SPEED = max_speed
        self.range_offset = range_offset
        self.track_width = track_width
        self._apply_adv(adv)
        self._set_speed_tiers()

    ADV_DEFAULTS = {
        'speed_scale': 0.6,
        'radius_max': 5.0,
        'nogap_fallback': False,
        'gap_hysteresis': 1.0,
        'disparity_enable': False,
        'disparity_thresh': 0.5,
        'car_half_width': 0.15,
        'disparity_margin': 0.10,
        'min_gap_width_m': 0.0,
        'speed_by_range': False,
        'speed_gain': 0.8,
        'speed_min': 0.8,
        'speed_cone_deg': 6.0,
        'lat_acc_max': 0.0,
        'wheelbase': 0.338,
    }

    def _apply_adv(self, adv: dict) -> None:
        for k, default in self.ADV_DEFAULTS.items():
            setattr(self, 'adv_' + k, adv.get(k, getattr(self, 'adv_' + k, default)))

    def _set_speed_tiers(self) -> None:
        scale = self.adv_speed_scale
        self.CORNERS_SPEED = 0.3 * self.MAX_SPEED * scale
        self.MILD_CORNERS_SPEED = 0.45 * self.MAX_SPEED * scale
        self.STRAIGHTS_SPEED = 0.8 * self.MAX_SPEED * scale
        self.ULTRASTRAIGHTS_SPEED = self.MAX_SPEED * scale

    def _preprocess_lidar(self, ranges) -> np.ndarray:
        """ 
        Preprocess the LiDAR scan array.

        This method performs preprocessing on the LiDAR scan array. The preprocessing steps include:
        1. Setting each value to the mean over a specified window.
        2. Rejecting high values (e.g., values greater than 3m).

        Parameters:
            ranges (numpy.ndarray): The LiDAR scan array.

        Returns:
            numpy.ndarray: The preprocessed LiDAR scan array.
        """
        self.radians_per_elem = (1.5 * np.pi) / len(ranges)
        self.n_beams = len(ranges)
        # we won't use the LiDAR data from directly behind us
        # full angle is -135 135
        # every point in the array is
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        # sets each value to the mean over a given window to smoothen the signal
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE)/self.PREPROCESS_CONV_SIZE, 'valid') 
        # clip the ranges between 0 and your maximum lidar distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        # reverse lidar because it is right to left
        return proc_ranges[::-1]

    def _get_steer_angle(self, point_x, point_y) -> float:
        """ 
        Get the angle of a particular element in the LiDAR data and
        transform it into an appropriate steering angle.
        
        Parameters:
            point_x (float): The x-coordinate of the LiDAR data point
            point_y (float): The y-coordinate of the LiDAR data point
        
        Returns:
            float: The transformed steering angle
        
        """
        steering_angle = math.atan2(point_y, point_x)
        return np.clip(steering_angle, -0.4, 0.4)

    def _get_best_range_point(self, proc_ranges) -> tuple:
        """ 
        Find the best point i.e. the middle of the largest gap within the bubble radius.
        
        Parameters:
            proc_ranges (list): List of processed ranges.
        
        Returns:
            tuple: The x and y coordinates of the best point.
        """
        #Get the bubble radius 
        radius = self._get_radius()
        
        #Find the largest gap
        gap_left, gap_right = self._find_largest_gap(ranges=proc_ranges, radius=radius)
        if self.adv_nogap_fallback and gap_right <= gap_left:
            far = int(np.argmax(proc_ranges))
            gap_left, gap_right = far, far + 1
        proc_middle = int((gap_left + gap_right) / 2)
        self.last_target_idx = proc_middle
        cone = max(1, int(round(np.radians(self.adv_speed_cone_deg) / self.radians_per_elem)))
        lo, hi = max(0, proc_middle - cone), min(len(proc_ranges), proc_middle + cone + 1)
        self.last_d_free = float(np.min(proc_ranges[lo:hi])) if hi > lo else 0.0
        center_correction = int(round(self.n_beams / 6))
        gap_left += self.range_offset - center_correction
        gap_right += self.range_offset - center_correction
        gap_middle = int((gap_right + gap_left) / 2)
        #Calculate cartesian point of the best point position from the lidar measurements in laser frame
        best_y = np.cos(gap_middle * self.radians_per_elem) * radius
        best_x = np.sin(gap_middle * self.radians_per_elem) * radius
        
        if self.DEBUG and self.best_gap.get_subscription_count() > 0:
            #Delete old gaps from RVIZ
            self._delete_gap_markers()

            #Visualise the gap
            gap_markers = MarkerArray()
            for i in range(gap_left, gap_right):
                mrk = Marker()
                mrk.header.frame_id = 'car_state/laser'
                mrk.header.stamp = self.get_clock().now().to_msg()
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.05
                mrk.scale.y = 0.05
                mrk.scale.z = 0.05
                mrk.color.a = 1.0
                mrk.color.r = 1.0
                mrk.color.g = 1.0
                mrk.id = int(i - gap_left)
                #Calculate cartesian point of the gap  marker position from the lidar measurements in laser frame
                mrk.pose.position.y = math.cos(i * self.radians_per_elem) * radius
                mrk.pose.position.x = math.sin(i * self.radians_per_elem) * radius
                mrk.pose.orientation.w = 1.0
                gap_markers.markers.append(mrk)
            self.best_gap.publish(gap_markers)

        if self.DEBUG and self.best_pnt.get_subscription_count() > 0:
            # visualize best point aka middle of the gap
            best_mrk = Marker()
            best_mrk.header.frame_id = 'car_state/laser'
            best_mrk.header.stamp = self.get_clock().now().to_msg()
            best_mrk.type = best_mrk.SPHERE
            best_mrk.scale.x = 0.2
            best_mrk.scale.y = 0.2
            best_mrk.scale.z = 0.2
            best_mrk.color.a = 1.0
            best_mrk.color.b = 1.0
            best_mrk.color.g = 1.0
            best_mrk.id = 0
            best_mrk.pose.position.y = best_y
            best_mrk.pose.position.x = best_x
            best_mrk.pose.orientation.w = 1.0
            self.best_pnt.publish(best_mrk)
        
        return best_x, best_y

    def process_lidar(self, ranges) -> tuple:
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm &
        calculate the speed and steering angle.

        Parameters:
            ranges (list): List of LiDAR scan ranges

        Returns:
            tuple: A tuple containing the speed and steering angle
        """
        #Preprocess the LiDAR to smoothen it
        proc_ranges = self._preprocess_lidar(ranges)
        
        proc_ranges = self._disparity_extend(proc_ranges) if self.adv_disparity_enable \
            else self._safety_border(proc_ranges)
        
        if self.DEBUG and self.scan_pub.get_subscription_count() > 0:
            scan_markers = MarkerArray()
            for i, scan in enumerate(proc_ranges):
                mrk = Marker()
                mrk.header.frame_id = 'car_state/laser'
                mrk.header.stamp = self.get_clock().now().to_msg()
                mrk.type = mrk.SPHERE
                mrk.scale.x = 0.05
                mrk.scale.y = 0.05
                mrk.scale.z = 0.05
                mrk.color.a = 1.0
                mrk.color.r = 1.0
                mrk.color.b = 1.0

                mrk.id = i
                mrk.pose.position.x = math.sin(i* self.radians_per_elem) * scan
                mrk.pose.position.y = math.cos(i* self.radians_per_elem) * scan
                mrk.pose.orientation.w = 1.0
                scan_markers.markers.append(mrk)
            self.scan_pub.publish(scan_markers)

        #Get best point to target aka middle of the largest gap
        best_x, best_y = self._get_best_range_point(proc_ranges)

        #Get steer angle from best points
        steering_angle = self._get_steer_angle(point_x=best_x, point_y=best_y)

        if self.mapping:
            speed = 1.5
        elif self.adv_speed_by_range:
            speed = float(np.clip(self.adv_speed_gain * (self.last_d_free or 0.0),
                                  self.adv_speed_min, self.MAX_SPEED * self.adv_speed_scale))
            if self.adv_lat_acc_max > 0.0 and abs(steering_angle) > 1e-3:
                v_lat = math.sqrt(self.adv_lat_acc_max * self.adv_wheelbase / math.tan(abs(steering_angle)))
                speed = max(self.adv_speed_min, min(speed, v_lat))
        else:
            if abs(steering_angle) > self.MILD_CURVE_ANGLE:
                speed = self.CORNERS_SPEED
            elif abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
                speed = self.MILD_CORNERS_SPEED
            elif abs(steering_angle) > self.ULTRASTRAIGHTS_ANGLE:
                speed = self.STRAIGHTS_SPEED
            else:
                speed = self.ULTRASTRAIGHTS_SPEED

        return speed, steering_angle

    def _find_largest_gap(self, ranges, radius) -> tuple:
        """ 
        Find the index of the starting and ending of the largest gap and its width

        Parameters:
            ranges (numpy.ndarray): Array of range values
            radius (float): Threshold radius value

        Returns:
            tuple: A tuple containing the index of the starting of the largest gap, 
                    the index of the ending of the largest gap, and the width of the largest gap.

        """
        #Binarise the ranges in zeros for values under the radius threshold and ones for above and equal
        bin_ranges = np.where(ranges >= radius, 1, 0)
        
        #Get largest gap from binary ranges
        bin_diffs = np.abs(np.diff(bin_ranges))
        bin_diffs[0] = 1
        bin_diffs[-1] = 1

        diff_idxs = bin_diffs.nonzero()[0]
        #Check that binarised ranges are positive
        high_gaps = []
        for i in range(len(diff_idxs)-1):
            low = diff_idxs[i]
            high = diff_idxs[i+1]
            high_gaps.append(np.mean(bin_ranges[low:high]) > 0.5)

        widths = np.diff(diff_idxs)
        scores = np.asarray(high_gaps, dtype=float) * widths
        if self.adv_min_gap_width_m > 0.0 and self.radians_per_elem is not None:
            for k in range(len(scores)):
                if scores[k] <= 0:
                    continue
                depth = float(np.min(ranges[diff_idxs[k]:diff_idxs[k + 1]]))
                if widths[k] * self.radians_per_elem * depth < self.adv_min_gap_width_m:
                    scores[k] = 0.0
        best = int(np.argmax(scores))
        if self.adv_gap_hysteresis > 1.0 and self.last_target_idx is not None and scores[best] > 0:
            for k in range(len(scores)):
                if scores[k] > 0 and diff_idxs[k] <= self.last_target_idx < diff_idxs[k + 1]:
                    if scores[best] < self.adv_gap_hysteresis * scores[k]:
                        best = k
                    break
        gap_left = diff_idxs[best]
        gap_width = int(scores[best])
        gap_right = gap_left + gap_width

        return gap_left, gap_right

    def _get_radius(self) -> float:
        """
        Calculate the radius based on the track width and velocity.

        Returns:
            float: The calculated radius.
        """
        # Empirically determined that this radius choosing makes sense
        return min(self.adv_radius_max, self.track_width / 2 + 2 * (self.velocity / self.MAX_SPEED))

    def set_vel(self, velocity) -> None:
        """
        Set the velocity of the car.
        
        Parameters:
            velocity (float): The desired velocity value.
        """
        self.velocity = velocity
    
    def _safety_border(self, ranges) -> np.ndarray:
        """
        Add a safety bubble if there is a big increase in the range between two points.

        Parameters:
            ranges (list): List of range values.

        Returns:
            np.ndarray: Array of filtered range values.
        """
        filtered = list(ranges)
        ranges_len = len(ranges)
        i = 0
        while i < ranges_len - 1:
            if ranges[i + 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i + j < ranges_len:
                        filtered[i + j] = ranges[i]
                i += self.SAFETY_RADIUS - 2
            i += 1
        # in other direction
        i = ranges_len - 1
        while i > 0:
            if ranges[i - 1] - ranges[i] > 0.5:
                for j in range(self.SAFETY_RADIUS):
                    if i - j >= 0:
                        filtered[i - j] = ranges[i]
                i = i - self.SAFETY_RADIUS + 2
            i -= 1
        return np.array(filtered)

    def _disparity_extend(self, ranges) -> np.ndarray:
        """
        Distance-based disparity extender. At every adjacent-beam jump > disparity_thresh,
        overwrite the far side with the near range for as many beams as
        (car_half_width + margin) subtends at the near range, so clearance is correct at
        every distance.
        """
        r = np.asarray(ranges, dtype=float)
        out = r.copy()
        n = len(r)
        half = self.adv_car_half_width + self.adv_disparity_margin
        jumps = np.diff(r)
        for i in np.nonzero(np.abs(jumps) > self.adv_disparity_thresh)[0]:
            if jumps[i] > 0:
                near = r[i]
                k = int(math.ceil(math.atan2(half, max(near, 1e-3)) / self.radians_per_elem))
                hi = min(n, i + 1 + k)
                out[i + 1:hi] = np.minimum(out[i + 1:hi], near)
            else:
                near = r[i + 1]
                k = int(math.ceil(math.atan2(half, max(near, 1e-3)) / self.radians_per_elem))
                lo = max(0, i + 1 - k)
                out[lo:i + 1] = np.minimum(out[lo:i + 1], near)
        return out

    def _delete_gap_markers(self) -> None:
        """
        Delete marker for rviz when not needed
        """
        del_mrk_array = MarkerArray()
        for i in range(1):
            del_mrk = Marker()
            del_mrk.header.frame_id = 'car_state/laser'
            del_mrk.header.stamp = self.get_clock().now().to_msg()
            del_mrk.action = del_mrk.DELETEALL
            del_mrk.id = i
            del_mrk_array.markers.append(del_mrk)
        self.best_gap.publish(del_mrk_array)
